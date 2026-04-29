#!/usr/bin/env python3
import time
import math
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

# ======================================
# 优化版 Native IMU 处理器
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
    self.is_calibrated = False

    # 状态缓存
    self.last_yaw = 0.0
    self.last_lat = 0.0
    self.last_lon = 0.0
    self.has_valid_data = False

    # 车辆几何参数 (由 carParams 获取)
    self.CP = None
    self.steerRatio = 15.0
    self.wheelbase = 2.8

  def update_calibration(self, cs):
    # 车辆静止且未打方向时，自动校准偏置
    if cs.vEgo < 0.1 and not cs.steeringPressed:
      raw_yaw = math.radians(cs.yawRate)
      self.yaw_bias = (self.yaw_bias * self.bias_count + raw_yaw) / (self.bias_count + 1)
      self.bias_count = min(self.bias_count + 1, 500) # 最多记录 5 秒数据
      if self.bias_count > 100:
        self.is_calibrated = True
    else:
      # 行驶中不再更新偏置，保持校准状态
      pass

  def main_loop(self):
    print("******** Optimized Native IMU Started (100Hz) ********")

    while True:
      self.sm.update(0)

      if self.CP is None and self.sm.updated['carParams']:
        self.CP = self.sm['carParams']
        self.steerRatio = self.CP.steerRatio
        self.wheelbase = self.CP.wheelbase

      if self.sm.updated['carState']:
        cs = self.sm['carState']
        v_ego = cs.vEgo

        # 1. 自动校准
        self.update_calibration(cs)

        # 2. 获取去偏后的原始值
        raw_yaw = math.radians(cs.yawRate) - self.yaw_bias

        # 3. 引入物理模型预测 (Kinematic Model)
        # 公式: yaw_rate = (v * tan(steer_angle / ratio)) / L
        # 物理模型对高频响应极快且没有噪声阶梯
        steer_angle_rad = math.radians(cs.steeringAngleDeg)
        if abs(v_ego) > 0.5:
          model_yaw = (v_ego * math.tan(steer_angle_rad / self.steerRatio)) / self.wheelbase
        else:
          model_yaw = 0.0

        # 4. 互补融合 (Complementary Filter)
        # alpha 越大越信任 IMU，越小越信任模型预测（更平滑，无延迟）
        # 高速时模型更准，增加模型权重
        alpha = float(np.interp(v_ego, [10, 30], [0.8, 0.4]))
        yaw = alpha * raw_yaw + (1.0 - alpha) * model_yaw

        # 5. 加速度处理
        lon = cs.aEgo
        lat = v_ego * yaw # 基于融合后的 yaw 计算侧向加速度，极其平滑

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
          'v': [-float(yaw), 0.0, 0.0], # 坐标系转换
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