#!/usr/bin/env python3
import math
import time
import random
import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

# 物理常数
G = 9.80665

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)
    self.frame = 0
    
    # 传感器偏置校准
    self.yaw_bias = None
    self.lat_bias = None
    self.bias_count = 0
    self.bias_stabled = False

  def update_calibration(self, cs):
    # 初始化偏置
    if self.yaw_bias is None and cs.vEgo < 0.1 and not cs.steeringPressed: 
      self.yaw_bias = cs.yawRate
      print(f"[IMU_INIT] Initialized Yaw Bias: {self.yaw_bias:.5f}")
    if self.lat_bias is None and cs.vEgo < 0.1 and not cs.steeringPressed: 
      self.lat_bias = cs.latAccel
      print(f"[IMU_INIT] Initialized Lat Bias: {self.lat_bias:.5f}")

    # 1. 静态校准
    if cs.vEgo < 0.01 and not cs.steeringPressed:
      self.bias_count += 1
      alpha = 1.0 / min(self.bias_count, 1000)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * cs.yawRate
      self.lat_bias = (1 - alpha) * self.lat_bias + alpha * cs.latAccel
      if self.bias_count > 500: self.bias_stabled = True
    
    # 2. 动态行驶校准
    elif cs.vEgo > 10.0 and abs(cs.steeringAngleDeg) < 5.0:
      self.yaw_bias = 0.999 * self.yaw_bias + 0.001 * cs.yawRate
      if self.frame % 1000 == 0:
        print(f"[IMU_CALIB] Adjusting Bias: {self.yaw_bias:.5f}, RawYaw: {cs.yawRate:.5f}")

  def main_loop(self):
    print("******** Native IMU V8.0 (JY62 Raw Data Mode) ********")
    
    while True:
      # 【核心黑科技2】：阻塞式读取取代轮询，消除 5~10ms 的软件相位抖动
      # 原代码使用 Ratekeeper(100) 和 sm.update(0) 轮询，这会导致和 carState 发布的 100Hz 产生节拍差。
      # 现在直接阻塞等待 carState，做到零延迟透传。
      self.sm.update(100)
      if not self.sm.updated['carState']:
        continue
        
      cs = self.sm['carState']
      v_ego = cs.vEgo
      self.update_calibration(cs)
      
      # 减去静态 Bias，且【绝不使用低通滤波】，避免引入任何相位延迟！
      clean_yaw = cs.yawRate - (self.yaw_bias if self.yaw_bias else 0.0)
      clean_lat = cs.latAccel - (self.lat_bias if self.lat_bias else 0.0)
      
      # 当缺乏有效侧向加速度且有速度时，仅做极基础的物理推导补充
      if abs(cs.latAccel) < 0.001 and v_ego > 1.0:
        clean_lat = v_ego * clean_yaw
        
      # 【核心黑科技】：注入极微小的高频白噪声
      # JY62 等物理传感器带有本底噪声，而原车 CAN 信号是平滑过滤过的。
      # 卡尔曼滤波 (locationd) 如果遇到绝对的 0.0 方差，矩阵求逆容易退化发散导致震荡。
      noise_wx = random.gauss(0.0, 0.002)
      noise_wy = random.gauss(0.0, 0.002)
      noise_wz = random.gauss(0.0, 0.001)
      
      noise_ax = random.gauss(0.0, 0.05)
      noise_ay = random.gauss(0.0, 0.05)
      noise_az = random.gauss(0.0, 0.05)
      
      # 限幅
      yaw_final = np.clip(clean_yaw + noise_wz, -2.0, 2.0)
      
      now_ns = time.monotonic_ns()
      
      # 陀螺仪: 映射后满足 locationd 变换 meas = [-v[2], -v[1], -v[0]] -> meas[2] = -v[0] = -yaw_final
      # 加入 noise_wy (Pitch Rate) 和 noise_wx (Roll Rate) 来提供自然的微小扰动
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': [float(yaw_final), float(noise_wy), float(noise_wx)], 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 加速度计: 映射后满足 meas[0] = lon, meas[1] = -lat, meas[2] = gravity
      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': [G + noise_az, float(clean_lat + noise_ay), -float(cs.aEgo + noise_ax)], 'status': 1}}
      self.pm.send('accelerometer', accel_msg)

      if self.frame % 1000 == 0:
        status = "Calibrating" if not self.bias_stabled else "Running"
        b_val = self.yaw_bias if self.yaw_bias is not None else 0.0
        print(f"[{status}] v={v_ego:.1f}, Bias={b_val:.5f}")

      self.frame += 1
      self.rk.keep_time()

def main():
  NativeIMUD().main_loop()

if __name__ == "__main__":
  main()