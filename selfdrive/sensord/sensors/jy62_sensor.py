#!/usr/bin/env python3
"""
JY62 IMU Sensor Driver
"""

import serial
import time
import struct
import math
from typing import Tuple
import cereal.messaging as messaging

from cereal import log
from openpilot.common.realtime import DT_DMON
from openpilot.system.hardware import TICI


class JY62Sensor:
  def __init__(self, device_path='/dev/ttyUSB0', baud_rate=115200):
    self.device_path = device_path
    self.baud_rate = baud_rate
    self.ser = None

    # 初始化消息发布器
    self.pm = messaging.PubMaster(['accelerometer', 'gyroscope'])

    # 数据包类型标识
    self.ACCEL_PKT = 0x51
    self.GYRO_PKT = 0x52
    self.ANGLE_PKT = 0x53

  def connect(self):
    """连接到JY62设备"""
    try:
      self.ser = serial.Serial(
        self.device_path,
        self.baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
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

  def _parse_packet(self, packet: bytes) -> Tuple[str, dict]:
    """解析JY62数据包"""
    if len(packet) != 11:
      return None, {}

    # 验证校验和
    checksum = sum(packet[:10]) & 0xFF
    if checksum != packet[10]:
      return None, {}

    packet_type = packet[1]
    timestamp = time.monotonic()

    if packet_type == self.ACCEL_PKT:
      # 解析加速度数据
      ax_raw, ay_raw, az_raw = struct.unpack('<hhh', packet[2:8])

      # 转换为m/s² (±16g量程)
      ax = (ax_raw / 32768.0) * 16.0 * 9.8
      ay = (ay_raw / 32768.0) * 16.0 * 9.8
      az = (az_raw / 32768.0) * 16.0 * 9.8

      return 'accelerometer', {
        'timestamp': timestamp,
        'ax': ax, 'ay': ay, 'az': az
      }

    elif packet_type == self.GYRO_PKT:
      # 解析角速度数据
      wx_raw, wy_raw, wz_raw = struct.unpack('<hhh', packet[2:8])

      # 转换为rad/s (±2000°/s量程)
      wx = (wx_raw / 32768.0) * 2000.0 * math.pi / 180.0
      wy = (wy_raw / 32768.0) * 2000.0 * math.pi / 180.0
      wz = (wz_raw / 32768.0) * 2000.0 * math.pi / 180.0

      return 'gyroscope', {
        'timestamp': timestamp,
        'wx': wx, 'wy': wy, 'wz': wz
      }

    return None, {}

  def update(self):
    """读取并处理数据"""
    if not self.ser or not self.ser.is_open:
      return

    # 读取数据
    data = self.ser.read(32)
    if not data:
      return

    # 缓冲区处理
    buffer = bytearray(data)

    # 查找并处理完整数据包
    while len(buffer) >= 11:
      # 查找包头
      header_index = -1
      for i in range(min(16, len(buffer))):
        if buffer[i] == 0x55:
          header_index = i
          break

      if header_index == -1:
        buffer = buffer[-10:] if len(buffer) > 10 else bytearray()
        break

      if header_index > 0:
        buffer = buffer[header_index:]

      if len(buffer) < 11:
        break

      # 提取数据包
      packet = buffer[:11]

      # 解析数据包
      msg_type, data = self._parse_packet(packet)

      # 发布消息
      if msg_type == 'accelerometer':
        # 创建加速度计消息
        dat = messaging.new_message('accelerometer', valid=True)
        dat.accelerometer = {
          'timestamp': int(data['timestamp'] * 1e9),
          'source': log.SensorEventData.SensorSource.jy62,
          'type': 1,
          'acceleration': {
            'v': [data['az'], data['ay'], -data['ax']],
            'status': 0
          }
        }

        # 发布消息
        self.pm.send('accelerometer', dat)

      elif msg_type == 'gyroscope':
        # 创建陀螺仪消息
        dat = messaging.new_message('gyroscope', valid=True)
        dat.gyroscope = {
          'timestamp': int(data['timestamp'] * 1e9),
          'source': log.SensorEventData.SensorSource.jy62,
          'type': 2,
          'gyroUncalibrated': {
            'v': [data['wz'], data['wy'], -data['wx']],
            'status': 0
          }
        }

        # 发布消息
        self.pm.send('gyroscope', dat)

      # 移除已处理的数据包
      buffer = buffer[11:]

def main():
  """主函数"""
  sensor = JY62Sensor()

  if not sensor.connect():
    return

  try:
    while True:
      sensor.update()
      time.sleep(0.001)  # 1ms延迟
  except KeyboardInterrupt:
    print("Stopping JY62 sensor...")
  finally:
    sensor.disconnect()

if __name__ == "__main__":
  main()