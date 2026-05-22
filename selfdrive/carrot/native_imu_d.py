#!/usr/bin/env python3
import math
import time
import random
import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.transformations.orientation import rot_from_euler

# 物理常数
G = 9.80665

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'liveCalibration'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)
    self.frame = 0
    self.device_from_calib = np.eye(3)
    
    # 传感器偏置校准
    self.yaw_bias = None
    self.lat_bias = None
    self.bias_count = 0
    self.bias_stabled = False

  def update_calibration(self, cs):
    # 零损耗强鲁棒初始化保护（即使在高速行车中途重启也能瞬间安全初始化并限幅）
    if self.yaw_bias is None:
      self.yaw_bias = np.clip(cs.yawRate, -0.05, 0.05)
      print(f"[IMU_INIT] Initialized Yaw Bias: {self.yaw_bias:.5f}")
    if self.lat_bias is None:
      self.lat_bias = np.clip(cs.latAccel, -0.5, 0.5)
      print(f"[IMU_INIT] Initialized Lat Bias: {self.lat_bias:.5f}")

    # 1. 静态校准（当车辆完全静止且方向盘无受力时进行，采用极低权重对齐，彻底隔离坡度和静止死区污染）
    if cs.vEgo < 0.01 and not cs.steeringPressed:
      self.bias_count += 1
      alpha = 1.0 / min(self.bias_count, 5000)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * cs.yawRate
      self.lat_bias = (1 - alpha) * self.lat_bias + alpha * cs.latAccel
      if self.bias_count > 500: 
        self.bias_stabled = True
    
    # 2. 动态高精行驶校准（只在高速且绝对直道 [方向盘转角 < 0.8度] 时进行，以超高时滞滤波器进行长期均值收敛）
    elif cs.vEgo > 15.0 and abs(cs.steeringAngleDeg) < 0.8:
      self.yaw_bias = 0.9998 * self.yaw_bias + 0.0002 * cs.yawRate
      self.lat_bias = 0.9998 * self.lat_bias + 0.0002 * cs.latAccel
      if self.frame % 1000 == 0:
        print(f"[IMU_CALIB] Dynamic Bias Adjusted - Yaw: {self.yaw_bias:.5f}, Lat: {self.lat_bias:.5f}")

    # 3. 终极硬件零偏保护阈值，硬性限幅防止任何意外发散
    self.yaw_bias = np.clip(self.yaw_bias, -0.05, 0.05)
    self.lat_bias = np.clip(self.lat_bias, -0.5, 0.5)

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
      
      # 更新设备到标定坐标系的旋转矩阵，解决 locationd 逆向校准导致的原车 IMU 旋转污染问题
      if self.sm.updated['liveCalibration']:
        calib = self.sm['liveCalibration'].rpyCalib
        if len(calib) == 3:
          self.device_from_calib = rot_from_euler(calib)
      
      # 减去静态 Bias，且【绝不使用低通滤波】，避免引入任何相位延迟！
      clean_yaw = cs.yawRate - (self.yaw_bias if self.yaw_bias else 0.0)
      clean_lat = cs.latAccel - (self.lat_bias if self.lat_bias else 0.0)
      
      # 当缺乏有效侧向加速度且有速度时，仅做极基础的物理推导补充
      if abs(cs.latAccel) < 0.001 and v_ego > 1.0:
        clean_lat = v_ego * clean_yaw
        
      # 【核心黑科技】：注入极微小的高频白噪声
      # JY62 等物理传感器带有本底噪声，而原车 CAN 信号是平滑过滤过的。
      # 卡尔曼滤波 (locationd) 如果遇到绝对 of 0.0 方差，矩阵求逆容易退化发散导致震荡。
      noise_wx = random.gauss(0.0, 0.002)
      noise_wy = random.gauss(0.0, 0.002)
      noise_wz = random.gauss(0.0, 0.001)
      
      noise_ax = random.gauss(0.0, 0.05)
      noise_ay = random.gauss(0.0, 0.05)
      noise_az = random.gauss(0.0, 0.05)
      
      # 限幅
      yaw_final = np.clip(clean_yaw + noise_wz, -2.0, 2.0)
      
      now_ns = time.monotonic_ns()
      
      # =========================================================================
      # 【超清准逆旋转标定对齐算法】
      # 解决 locationd 内置 EKF 将原车 IMU 当作手机 IMU 并用 rpyCalib 进行旋转变换造成的坐标扭曲！
      # =========================================================================
      
      # 1. 陀螺仪逆旋转：
      # 车身坐标系下的测量向量为 meas_car = [-noise_wx, -noise_wy, -yaw_final]
      # 映射到设备坐标系为 meas_device = R_device_from_calib * meas_car
      meas_car_gyro = np.array([-float(noise_wx), -float(noise_wy), -float(yaw_final)])
      meas_device_gyro = self.device_from_calib @ meas_car_gyro
      v_gyro = [-float(meas_device_gyro[2]), -float(meas_device_gyro[1]), -float(meas_device_gyro[0])]

      # 2. 加速度计逆旋转：
      # 车身坐标系下的测量向量为 meas_car = [cs.aEgo + noise_ax, -clean_lat - noise_ay, -G - noise_az]
      meas_car_accel = np.array([float(cs.aEgo + noise_ax), -float(clean_lat + noise_ay), -float(G + noise_az)])
      meas_device_accel = self.device_from_calib @ meas_car_accel
      v_accel = [-float(meas_device_accel[2]), -float(meas_device_accel[1]), -float(meas_device_accel[0])]

      # 陀螺仪: 映射后满足 locationd 变换 meas = [-v[2], -v[1], -v[0]] -> meas[2] = -v[0] = -yaw_final
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': v_gyro, 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 加速度计: 映射后满足 meas[0] = lon, meas[1] = -lat, meas[2] = gravity
      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': v_accel, 'status': 1}}
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