#!/usr/bin/env python3
import math
import time
import numpy as np
from collections import deque
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, DT_MDL
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

# 重力加速度常数
G = 9.80665

# ======================================
# V6 Omniscient IMU (LSM6DS3 Logic Refactored)
# ======================================

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'carParams', 'carrotMan', 'liveParameters'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    # 核心状态机
    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)
    
    # 滤波与校准状态
    self.yaw_bias = 0.0
    self.bias_learned = False
    self.bias_count = 0
    self.yaw_history = deque(maxlen=5)
    
    # 动态滤波器
    self.yaw_filter = FirstOrderFilter(0.0, 0.05, self.dt)
    self.accel_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.nav_curvature_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.yaw_accel_filter = FirstOrderFilter(0.0, 0.1, self.dt) # 100ms 过滤微分噪声
    
    # 历史状态
    self.last_yaw = 0.0
    self.has_valid_data = False
    
    # 车辆模型状态
    sr = self.params.get_float("LatMpcSteerRatio") if self.params.get("LatMpcSteerRatio") else 15.0
    self.learned_steer_ratio = max(13.0, min(20.0, sr))
    self.rls_P = 10.0
    self.rls_lambda = 0.999
    self.model_yaw_lag = 0.0

  def update_calibration(self, cs):
    v_ego = cs.vEgo
    raw_yaw = cs.yawRate
    
    # 1. 深度静态校准 (停车时快速学习)
    if v_ego < 0.05 and not cs.steeringPressed:
      self.bias_count += 1
      alpha = 1.0 / min(self.bias_count, 500)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * raw_yaw
      self.bias_learned = True
    
    # 2. 动态温漂补偿 (直线行驶时极弱学习)
    # 必须参考 angle_offset，否则有物理偏置的车永远学不到
    angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
    if v_ego > 15.0 and abs(cs.steeringAngleDeg - angle_offset) < 0.3:
      self.yaw_bias = 0.99995 * self.yaw_bias + 0.00005 * raw_yaw

  def process_imu(self, cs):
    v_ego = cs.vEgo
    
    # 1. 获取原始值并减去偏置
    raw_yaw = cs.yawRate - self.yaw_bias
    raw_accel = cs.aEgo
    
    # 2. 静止锁定逻辑 (Stationary Lock) - 防止静止漂移
    if v_ego < 0.05:
      return 0.0, 0.0
      
    # 3. 加速度计重力补偿
    # 利用 pitch 数据修正 aEgo。 a_true = a_measured + G * sin(pitch)
    pitch = self.sm['liveParameters'].pitch if self.sm.seen['liveParameters'] else 0.0
    corrected_accel = raw_accel + G * math.sin(pitch)
    
    # 4. 非线性噪声抑制
    if abs(raw_yaw) < 0.0005:
      raw_yaw = 0.0
    
    # 5. 中值滤波剔除尖峰
    self.yaw_history.append(raw_yaw)
    if len(self.yaw_history) >= 3:
      med_yaw = sorted(list(self.yaw_history))[len(self.yaw_history)//2]
    else:
      med_yaw = raw_yaw
      
    return med_yaw, corrected_accel

  def learn_parameters(self, cs, raw_yaw):
    v_ego = cs.vEgo
    steer_angle = math.radians(cs.steeringAngleDeg)
    wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
    if abs(v_ego) > 10.0 and abs(steer_angle) > 0.05 and abs(raw_yaw) > 0.02:
        instant_ratio = (v_ego * math.tan(steer_angle)) / (raw_yaw * wheelbase)
        if 10.0 < instant_ratio < 25.0:
            error = instant_ratio - self.learned_steer_ratio
            self.rls_P = self.rls_P / self.rls_lambda
            gain = self.rls_P / (1 + self.rls_P)
            self.learned_steer_ratio += gain * error
            self.rls_P = (1 - gain) * self.rls_P
            self.learned_steer_ratio = max(13.0, min(20.0, self.learned_steer_ratio))

  def main_loop(self):
    print("******** Omniscient IMU V6 Refactored (LSM6DS3 Logic) ********")
    
    while True:
      self.sm.update(0)
      if not self.sm.updated['carState']:
        self.rk.keep_time()
        continue
        
      cs = self.sm['carState']
      self.update_calibration(cs)
      
      # 核心处理 (包含偏置和重力补偿)
      clean_yaw, clean_accel = self.process_imu(cs)
      
      # 在线参数学习
      self.learn_parameters(cs, clean_yaw)
      
      # 引入转向角偏置补偿
      angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
      steer_angle = math.radians(cs.steeringAngleDeg - angle_offset)
      wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
      
      # 物理模型预测
      if abs(cs.vEgo) > 0.5:
        target_model_yaw = (cs.vEgo * math.tan(steer_angle / self.learned_steer_ratio)) / wheelbase
      else:
        target_model_yaw = 0.0
        
      # 导航先验融合
      nav_curvature = 0.0
      if self.sm.updated['carrotMan']:
        nav_curvature = self.nav_curvature_filter.update(self.sm['carrotMan'].atcCurvature)
      nav_yaw_rate = cs.vEgo * nav_curvature

      # 卡尔曼融合估计 (包含相位超前补偿)
      trust_sensor = np.interp(cs.vEgo, [5.0, 15.0], [0.6, 0.9])
      yaw_fusion = trust_sensor * clean_yaw + (1.0 - trust_sensor) * target_model_yaw
      yaw_fusion = 0.8 * yaw_fusion + 0.2 * nav_yaw_rate # 引入 20% 导航先验
      
      # 80ms 相位超前补偿 (抵消传感器到控制器的总延迟)
      # 增加一阶滤波防止微分项放大高频噪声
      yaw_accel_raw = (yaw_fusion - self.last_yaw) * self.freq
      yaw_accel = self.yaw_accel_filter.update(yaw_accel_raw)
      self.last_yaw = yaw_fusion
      yaw_final = yaw_fusion + yaw_accel * 0.08

      # 发布消息 (匹配 locationd 坐标系 [-v2, -v1, -v0])
      now_ns = time.monotonic_ns()
      
      # 发布陀螺仪
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 发布加速度计
      accel_msg = messaging.new_message('accelerometer', valid=True)
      lat_accel = cs.vEgo * yaw_final
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': [0.0, -float(lat_accel), -float(clean_accel)], 'status': 1}}
      self.pm.send('accelerometer', accel_msg)

      if self.rk.frame % 1000 == 0:
        print(f"IMU: v={cs.vEgo:.1f}, yaw={yaw_final:.4f}, ratio={self.learned_steer_ratio:.2f}, bias={self.yaw_bias:.5f}")

      self.rk.keep_time()

if __name__ == "__main__":
  NativeIMUD().main_loop()