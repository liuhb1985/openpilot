#!/usr/bin/env python3
import time
import math
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

# ======================================
# 高级优化版 Native IMU 处理器 (V2 - 稳重型)
# ======================================

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'carParams'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    # 核心参数
    self.freq = 100.0
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)

    # 校准与偏置
    self.yaw_bias = 0.0
    self.bias_count = 0
    
    # 状态缓存
    self.last_yaw = 0.0
    self.last_lat = 0.0
    self.last_lon = 0.0
    self.has_valid_data = False

    # 车辆几何参数
    self.CP = None
    self.steerRatio = 15.0
    self.wheelbase = 2.8

    # 滤波状态 (二阶滤波)
    self.yaw_filtered = 0.0
    self.yaw_rate_prev = 0.0

  def update_calibration(self, cs):
    # 车辆静止且未打方向时，校准偏置
    if cs.vEgo < 0.05 and not cs.steeringPressed:
      raw_yaw = math.radians(cs.yawRate)
      self.yaw_bias = (self.yaw_bias * self.bias_count + raw_yaw) / (self.bias_count + 1)
      self.bias_count = min(self.bias_count + 1, 1000)
    else:
      pass

  def main_loop(self):
    print("******** Advanced Native IMU V2 Started (Steady Mode) ********")

    while True:
      self.sm.update(0)

      if self.CP is None and self.sm.updated['carParams']:
        self.CP = self.sm['carParams']
        self.steerRatio = self.CP.steerRatio
        self.wheelbase = self.CP.wheelbase

      if self.sm.updated['carState']:
        cs = self.sm['carState']
        v_ego = cs.vEgo
        steer_angle = cs.steeringAngleDeg
        
        # 1. 校准
        self.update_calibration(cs)

        # 2. 获取原始数据并去偏
        raw_yaw = math.radians(cs.yawRate) - self.yaw_bias
        
        # 3. 动态死区 (抑制静止时的微小抖动)
        if abs(raw_yaw) < 0.0005: # 约 0.03 deg/s
          raw_yaw = 0.0

        # 4. 物理模型预测
        steer_angle_rad = math.radians(steer_angle)
        if abs(v_ego) > 0.3:
          model_yaw = (v_ego * math.tan(steer_angle_rad / self.steerRatio)) / self.wheelbase
        else:
          model_yaw = 0.0

        # 5. [核心优化] 直道稳定增强逻辑 (Straight-line Stability)
        # 如果方向盘角度很小，大幅度增加模型权重，强行锁定直线
        if abs(steer_angle) < 0.8:
          alpha = 0.1 # 90% 信任模型 (模型在直道就是 0)
        else:
          # 弯道中动态调整融合权重
          alpha = float(np.interp(v_ego, [10, 30], [0.7, 0.4]))

        # 6. 互补融合
        yaw = alpha * raw_yaw + (1.0 - alpha) * model_yaw

        # 7. [核心优化] 二阶平滑滤波 (抑制方向盘微调反馈)
        # 使用低通滤波防止角速度变化过快导致的转向震荡
        self.yaw_filtered = 0.3 * yaw + 0.7 * self.yaw_filtered
        yaw = self.yaw_filtered

        # 8. 加速度处理
        lon = cs.aEgo
        lat = v_ego * yaw

        # 异常保护
        if abs(yaw) > 5.0: yaw = self.last_yaw
        if abs(lon) > 20.0: lon = self.last_lon
        if abs(lat) > 20.0: lat = self.last_lat

        self.last_yaw, self.last_lon, self.last_lat = yaw, lon, lat
        self.has_valid_data = True

      elif self.has_valid_data:
        yaw, lon, lat = self.last_yaw, self.last_lon, self.last_lat
      else:
        self.rk.keep_time()
        continue

      # 发布数据
      now_ns = time.monotonic_ns()

      # Gyroscope
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {
        'timestamp': now_ns,
        'type': 16,
        'sensor': 5,
        'gyroUncalibrated': {
          'v': [-float(yaw), 0.0, 0.0],
          'status': 1
        }
      }
      self.pm.send('gyroscope', gyro_msg)

      # Accelerometer
      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {
        'timestamp': now_ns,
        'type': 1,
        'sensor': 1,
        'acceleration': {
          'v': [0.0, -float(lat), -float(lon)],
          'status': 1
        }
      }
      self.pm.send('accelerometer', accel_msg)

      self.rk.keep_time()

if __name__ == "__main__":
  imud = NativeIMUD()
  imud.main_loop()