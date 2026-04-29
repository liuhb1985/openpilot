#!/usr/bin/env python3
import time
import math
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

# ======================================
# 终极优化版 Native IMU 处理器 (V4 - 全感官融合型)
# ======================================

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'carParams'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    # 核心参数
    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)

    # 状态缓存
    self.yaw_bias = 0.0
    self.bias_count = 0
    self.last_yaw = 0.0
    self.last_lat = 0.0
    self.last_lon = 0.0
    self.has_valid_data = False

    # 车辆几何参数 (初始化默认值)
    self.CP = None
    self.steerRatio = 15.0
    self.wheelbase = 2.8
    self.trackWidth = 1.6 # 轮距
    self.lever_arm_offset = 1.5 # IMU 相对于后轴的前向偏移距离 (估计值)

    # 滤波与预测状态
    self.yaw_filtered = 0.0
    self.yaw_accel = 0.0

  def update_calibration(self, cs):
    if cs.vEgo < 0.05 and not cs.steeringPressed:
      raw_yaw = math.radians(cs.yawRate)
      self.yaw_bias = (self.yaw_bias * self.bias_count + raw_yaw) / (self.bias_count + 1)
      self.bias_count = min(self.bias_count + 1, 1000)

  def main_loop(self):
    print("******** Final-Grade Native IMU V4 Started (Full Fusion) ********")

    while True:
      self.sm.update(0)

      if self.CP is None and self.sm.updated['carParams']:
        self.CP = self.sm['carParams']
        self.steerRatio = self.CP.steerRatio
        self.wheelbase = self.CP.wheelbase
        # 尝试从参数获取真实轮距
        self.trackWidth = getattr(self.CP, 'wheelStep', 1.6) 

      if self.sm.updated['carState']:
        cs = self.sm['carState']
        v_ego = cs.vEgo
        steer_angle = cs.steeringAngleDeg
        steer_rate = cs.steeringRateDeg
        
        # 1. 自动校准
        self.update_calibration(cs)

        # 2. 数据获取与去偏
        raw_yaw = math.radians(cs.yawRate) - self.yaw_bias
        
        # 3. [终极优化] 轮速差辅助 Yaw Rate (如果可用)
        # 轮速差对于低频转动极其精确，是防漂移的终极手段
        wheel_yaw = 0.0
        has_wheel_speed = False
        if hasattr(cs, 'wheelSpeeds') and v_ego > 1.0:
            # 使用后轮转速差计算 (不受转向影响)
            try:
                ws = cs.wheelSpeeds
                # Yaw = (Vr - Vl) / TrackWidth
                wheel_yaw = (ws.rl - ws.rr) / self.trackWidth
                has_wheel_speed = True
            except Exception:
                pass

        # 4. 非线性转向补偿预测
        steer_factor = 1.0 + (abs(steer_angle) * 0.0005) 
        model_yaw = 0.0
        if abs(v_ego) > 0.3:
          model_yaw = (v_ego * math.tan(math.radians(steer_angle) / (self.steerRatio * steer_factor))) / self.wheelbase

        # 5. 三源动态融合 (IMU + Model + Wheel)
        intent_factor = np.interp(abs(steer_rate), [0, 50], [0.0, 0.3])
        # 基础权重
        alpha_imu = 0.6 + intent_factor
        alpha_model = 0.3
        alpha_wheel = 0.1 if has_wheel_speed else 0.0
        
        # 归一化权重
        total_w = alpha_imu + alpha_model + alpha_wheel
        yaw = (alpha_imu * raw_yaw + alpha_model * model_yaw + alpha_wheel * wheel_yaw) / total_w

        # 6. 直道高精度锁定
        if abs(steer_angle) < 0.7:
          yaw = 0.05 * yaw + 0.95 * model_yaw # 在直道几乎完全相信模型和轮速平衡

        # 7. 相位超前预测 (25ms)
        current_yaw_accel = (yaw - self.last_yaw) / self.dt
        self.yaw_accel = 0.15 * current_yaw_accel + 0.85 * self.yaw_accel
        yaw_predicted = yaw + self.yaw_accel * 0.025

        # 8. 二阶自适应滤波
        self.yaw_filtered = 0.35 * yaw_predicted + 0.65 * self.yaw_filtered
        yaw_final = self.yaw_filtered

        # 9. [终极优化] 杠杆臂补偿 (Lever Arm Compensation)
        # 修正侧向加速度计因为安装位置不在重心而产生的偏差
        # Lat_accel_corrected = Lat_accel - (Yaw_accel * Offset)
        lon = cs.aEgo
        raw_lat = v_ego * yaw_final
        lat_corrected = raw_lat - (self.yaw_accel * self.lever_arm_offset * 0.1) # 适度补偿
        
        # 异常保护
        if abs(yaw_final) > 5.0: yaw_final = self.last_yaw
        
        self.last_yaw, self.last_lon, self.last_lat = yaw_final, lon, lat_corrected
        self.has_valid_data = True

      elif self.has_valid_data:
        yaw_final, lon, lat_corrected = self.last_yaw, self.last_lon, self.last_lat
      else:
        self.rk.keep_time()
        continue

      # 发布消息
      now_ns = time.monotonic_ns()
      
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {
        'timestamp': now_ns,
        'type': 16,
        'sensor': 5,
        'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}
      }
      self.pm.send('gyroscope', gyro_msg)

      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {
        'timestamp': now_ns,
        'type': 1,
        'sensor': 1,
        'acceleration': {'v': [0.0, -float(lat_corrected), -float(lon)], 'status': 1}
      }
      self.pm.send('accelerometer', accel_msg)

      self.rk.keep_time()

if __name__ == "__main__":
  imud = NativeIMUD()
  imud.main_loop()