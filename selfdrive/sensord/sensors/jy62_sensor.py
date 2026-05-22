#!/usr/bin/env python3
"""
JY62 IMU Sensor Driver - Ultra Low Latency Edition
"""

import serial
import time
import struct
import math
from typing import Tuple
import cereal.messaging as messaging

from cereal import log
from openpilot.common.realtime import DT_DMON

# ==========================================
# 物理挂载方向配置 (MOUNT_ORIENTATION)
# ==========================================
# 'FLAT' : 模块平放 (Z轴朝天，X轴朝前)。默认的原始姿态。
# 'VERTICAL' : 模块竖放 (Z轴朝向驾驶员，Y轴朝天，X轴朝右)。与 LSM6DS3 官方设备一致的姿态。
# 'SIDE_X_UP' : 模块侧放 (Y轴朝向车身左侧，X轴朝天，Z轴朝向驾驶员)。
# 'SIDE_X_DOWN' : 模块侧放 (Y轴朝向车身左侧，X轴朝地，Z轴朝向车头前方)。
MOUNT_ORIENTATION = 'FLAT'

class JY62Sensor:
  def __init__(self, device_path='/dev/ttyUSB0', baud_rate=115200):
    self.device_path = device_path
    self.baud_rate = baud_rate
    self.ser = None

    # 持久化缓冲区，防止断包和丢包
    self.buffer = bytearray()

    # 初始化消息发布器
    self.pm = messaging.PubMaster(['accelerometer', 'gyroscope'])

    # 数据包类型标识
    self.ACCEL_PKT = 0x51
    self.GYRO_PKT = 0x52
    self.ANGLE_PKT = 0x53
    
    self.last_accel_print_time = 0.0
    self.last_gyro_print_time = 0.0

  def connect(self):
    """连接到JY62设备"""
    try:
      self.ser = serial.Serial(
        self.device_path,
        self.baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        # 设置极小的 timeout 保证 read 操作能响应硬件热拔插，但依然保持阻塞特性
        timeout=1.0 
      )
      print(f"Connected to JY62 at {self.device_path}")
      return True
    except Exception as e:
      print(f"Failed to connect to JY62: {e}")
      return False

  def disconnect(self):
    """断开与JY62设备的连接"""
    if self.ser and self.ser.is_open:
      self.ser.close()

  def _parse_packet(self, packet: bytes, timestamp: float) -> Tuple[str, dict]:
    """解析JY62数据包"""
    if len(packet) != 11:
      return None, {}

    # 验证校验和
    checksum = sum(packet[:10]) & 0xFF
    if checksum != packet[10]:
      return None, {}

    packet_type = packet[1]

    if packet_type == self.ACCEL_PKT:
      # 解析加速度数据
      ax_raw, ay_raw, az_raw = struct.unpack('<hhh', packet[2:8])

      # 转换为m/s² (±16g量程)
      ax = (ax_raw / 32768.0) * 16.0 * 9.8
      ay = (ay_raw / 32768.0) * 16.0 * 9.8
      az = (az_raw / 32768.0) * 16.0 * 9.8

      # 根据物理摆放方向进行坐标系映射
      if MOUNT_ORIENTATION == 'VERTICAL':
        mapped_v = [ay, -ax, az]
      elif MOUNT_ORIENTATION == 'SIDE_X_UP':
        # 侧放：X朝天，Z朝后，Y朝左
        mapped_v = [ax, ay, az]
      elif MOUNT_ORIENTATION == 'SIDE_X_DOWN':
        # 侧放：X朝地，Z朝前，Y朝左
        mapped_v = [-ax, ay, -az]
      else:
        # 平放 (默认)：Z朝天，X朝前，Y朝左
        mapped_v = [az, ay, -ax]

      return 'accelerometer', {
        'timestamp': timestamp,
        'ax': ax, 'ay': ay, 'az': az,
        'v': mapped_v
      }

    elif packet_type == self.GYRO_PKT:
      # 解析角速度数据
      wx_raw, wy_raw, wz_raw = struct.unpack('<hhh', packet[2:8])

      # 转换为rad/s (±2000°/s量程)
      wx = (wx_raw / 32768.0) * 2000.0 * math.pi / 180.0
      wy = (wy_raw / 32768.0) * 2000.0 * math.pi / 180.0
      wz = (wz_raw / 32768.0) * 2000.0 * math.pi / 180.0

      if MOUNT_ORIENTATION == 'VERTICAL':
        mapped_v = [wy, -wx, wz]
      elif MOUNT_ORIENTATION == 'SIDE_X_UP':
        mapped_v = [wx, wy, wz]
      elif MOUNT_ORIENTATION == 'SIDE_X_DOWN':
        mapped_v = [-wx, wy, -wz]
      else:
        mapped_v = [wz, wy, -wx]

      return 'gyroscope', {
        'timestamp': timestamp,
        'wx': wx, 'wy': wy, 'wz': wz,
        'v': mapped_v
      }

    return None, {}

  def update(self):
    """读取并处理数据 (极限低延迟阻塞架构)"""
    if not self.ser or not self.ser.is_open:
      time.sleep(0.1) # 若掉线则避免死循环占满CPU
      return

    # 【极限优化1】：内核级阻塞唤醒
    # 使用 max(1, in_waiting) 强制系统挂起，直到硬件总线上有第1个字节到达才瞬间唤醒。
    # 彻底告别原本 read(32) 死等 30ms 造成的包堆积和延迟！
    to_read = max(1, self.ser.in_waiting)
    data = self.ser.read(to_read)
    
    # 【极限优化2】：高精度截取前沿时间戳
    # 只要操作系统从串口拿到数据，立刻打下时间戳，这是最贴近物理运动发生的时间！
    current_timestamp = time.monotonic()

    if not data:
      return

    # 追加到持久化缓冲区
    self.buffer.extend(data)

    # 查找并处理完整数据包
    while len(self.buffer) >= 11:
      # 查找包头 0x55
      header_index = -1
      for i in range(len(self.buffer)):
        if self.buffer[i] == 0x55:
          header_index = i
          break

      # 如果没找到包头，说明全都是无效数据，清空
      if header_index == -1:
        self.buffer.clear()
        break

      # 将包头对齐到缓冲区起始位置
      if header_index > 0:
        self.buffer = self.buffer[header_index:]

      # 对齐后如果长度不足一个包，等待下一次读取
      if len(self.buffer) < 11:
        break

      # 提取整整 11 字节的数据包
      packet = self.buffer[:11]

      # 解析数据包，传入精确的前沿时间戳
      msg_type, parsed_data = self._parse_packet(packet, current_timestamp)

      # 发布消息
      if msg_type == 'accelerometer':
        dat = messaging.new_message('accelerometer', valid=True)
        dat.accelerometer = {
          'timestamp': int(parsed_data['timestamp'] * 1e9),
          'source': log.SensorEventData.SensorSource.jy62,
          'sensor': 1, # SENSOR_ACCELEROMETER
          'type': 1,   # SENSOR_TYPE_ACCELEROMETER
          'acceleration': {
            'v': parsed_data['v'],
            'status': 0
          }
        }
        self.pm.send('accelerometer', dat)
        
        # 为了不刷屏，控制打印频率为10秒1次
        if current_timestamp - self.last_accel_print_time >= 10.0:
          print(f"JY62 Accel: {parsed_data['ax']:.4f}, {parsed_data['ay']:.4f}, {parsed_data['az']:.4f}")
          self.last_accel_print_time = current_timestamp

      elif msg_type == 'gyroscope':
        dat = messaging.new_message('gyroscope', valid=True)
        dat.gyroscope = {
          'timestamp': int(parsed_data['timestamp'] * 1e9),
          'source': log.SensorEventData.SensorSource.jy62,
          'sensor': 5, # SENSOR_GYRO_UNCALIBRATED
          'type': 16,  # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
          'gyroUncalibrated': {
            'v': parsed_data['v'],
            'status': 0
          }
        }
        self.pm.send('gyroscope', dat)
        
        if current_timestamp - self.last_gyro_print_time >= 10.0:
          print(f"JY62 Gyro: {parsed_data['wx']:.4f}, {parsed_data['wy']:.4f}, {parsed_data['wz']:.4f}")
          self.last_gyro_print_time = current_timestamp

      else:
        # 如果校验和失败或包类型未知，可能碰巧遇到了数据流中的伪 0x55
        # 我们只向后滑动 1 个字节，重新寻找真正的包头，避免错杀有效数据
        self.buffer = self.buffer[1:]
        continue

      # 【极限优化3】：安全切割缓冲区
      # 从缓冲区移除已处理的11字节有效数据包，留下剩余的碎包给下一次循环，彻底解决丢包断裂 Bug。
      self.buffer = self.buffer[11:]

def main():
  """主函数"""
  sensor = JY62Sensor()

  if not sensor.connect():
    return

  try:
    print("JY62 Driver Started (Ultra-Low Latency Mode)")
    while True:
      # 【极限优化4】：移除破坏节奏的软件 Sleep
      # 完全依靠 serial.read 的硬件级中断阻塞，把 CPU 让给系统，同时做到真正的0延迟。
      sensor.update()
  except KeyboardInterrupt:
    print("Stopping JY62 sensor...")
  finally:
    sensor.disconnect()

if __name__ == "__main__":
  main()