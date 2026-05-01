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
# V6 Omniscient IMU (Final Soul-Matched Edition)
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
    self.frame = 0
    
    # 滤波与校准状态 (双通道)
    self.yaw_bias = 0.0
    self.lat_bias = 0.0
    self.bias_count = 0
    self.yaw_history = deque(maxlen=5)
    self.lat_history = deque(maxlen=5)
    
    # 动态滤波器
    self.yaw_filter = FirstOrderFilter(0.0, 0.05, self.dt)
    self.accel_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.yaw_accel_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.nav_curvature_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    
    # 历史状态
    self.last_yaw = 0.0
    self.has_valid_data = False
    
    # 车辆模型状态 (转向比在线学习)
    sr = self.params.get_float("LatMpcSteerRatio") if self.params.get("LatMpcSteerRatio") else 15.0
    self.learned_steer_ratio = max(13.0, min(20.0, sr))
    self.rls_P = 10.0
    self.rls_lambda = 0.999

  def update_calibration(self, cs):
    v_ego = cs.vEgo
    angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
    
    # 1. 深度静态校准 (停车且稳定时)
    if v_ego < 0.05 and not cs.steeringPressed:
      self.bias_count += 1
      alpha = 1.0 / min(self.bias_count, 1000)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * cs.yawRate
      self.lat_bias = (1 - alpha) * self.lat_bias + alpha * cs.latAccel
      self.has_valid_data = True
    
    # 2. 动态零点补偿 (直线行驶且方向盘正中时，弱学习抗漂移)
    elif v_ego > 15.0 and abs(cs.steeringAngleDeg - angle_offset) < 0.2:
      # 极弱学习率，确保不影响正常驾驶
      self.yaw_bias = 0.99995 * self.yaw_bias + 0.00005 * cs.yawRate
      self.lat_bias = 0.99995 * self.lat_bias + 0.00005 * cs.latAccel

  def process_imu(self, cs):
    v_ego = cs.vEgo
    
    # 1. 获取并减去偏置 (实现真正的传感器对齐)
    raw_yaw = cs.yawRate - self.yaw_bias
    raw_lat = cs.latAccel - self.lat_bias
    raw_accel = cs.aEgo
    
    # 2. 静止锁定 (防止静止时的数值漂移)
    if v_ego < 0.05:
      return 0.0, 0.0, 0.0
      
    # 3. 纵向重力补偿 (坡度修正)
    pitch = self.sm['liveParameters'].pitch if self.sm.seen['liveParameters'] else 0.0
    corrected_accel = raw_accel + G * math.sin(pitch)
    
    # 4. 中值滤波 (消除 CAN 总线数据毛刺)
    self.yaw_history.append(raw_yaw)
    self.lat_history.append(raw_lat)
    
    if len(self.yaw_history) >= 3:
      med_yaw = sorted(list(self.yaw_history))[len(self.yaw_history)//2]
      med_lat = sorted(list(self.lat_history))[len(self.lat_history)//2]
    else:
      med_yaw = raw_yaw
      med_lat = raw_lat
      
    return med_yaw, med_lat, corrected_accel

  def learn_parameters(self, cs, clean_yaw):
    # RLS 在线转向比学习逻辑
    v_ego = cs.vEgo
    steer_angle = math.radians(cs.steeringAngleDeg)
    wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
    if abs(v_ego) > 10.0 and abs(steer_angle) > 0.05 and abs(clean_yaw) > 0.02:
        instant_ratio = (v_ego * math.tan(steer_angle)) / (clean_yaw * wheelbase)
        if 10.0 < instant_ratio < 25.0:
            error = instant_ratio - self.learned_steer_ratio
            self.rls_P = self.rls_P / self.rls_lambda
            gain = self.rls_P / (1 + self.rls_P)
            self.learned_steer_ratio += gain * error
            self.rls_P = (1 - gain) * self.rls_P
            self.learned_steer_ratio = max(13.0, min(20.0, self.learned_steer_ratio))

  def main_loop(self):
    print("******** Omniscient IMU V6 Final (Soul-Matched logic) ********")
    
    while True:
      self.sm.update(0)
      if not self.sm.updated['carState']:
        self.rk.keep_time()
        continue
        
      cs = self.sm['carState']
      self.update_calibration(cs)
      
      # 1. 传感器信号处理 (偏置、滤波、重力补偿)
      clean_yaw, clean_lat, clean_accel = self.process_imu(cs)
      
      # 2. 在线参数演进
      self.learn_parameters(cs, clean_yaw)
      
      # 3. 物理模型预测 (包含 angleOffset 偏置)
      angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
      steer_angle_rad = math.radians(cs.steeringAngleDeg - angle_offset)
      wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
      
      if abs(cs.vEgo) > 0.5:
        target_model_yaw = (cs.vEgo * math.tan(steer_angle_rad / self.learned_steer_ratio)) / wheelbase
      else:
        target_model_yaw = 0.0
        
      # 4. 导航先验增强
      nav_curvature = 0.0
      if self.sm.updated['carrotMan']:
        nav_curvature = self.nav_curvature_filter.update(self.sm['carrotMan'].atcCurvature)
      nav_yaw_rate = cs.vEgo * nav_curvature

      # 5. 卡尔曼级融合估计
      # 策略：低速信任模型，高速信任传感器，并融入 20% 导航路径前馈
      trust_sensor = np.interp(cs.vEgo, [5.0, 15.0], [0.6, 0.9])
      yaw_fusion = trust_sensor * clean_yaw + (1.0 - trust_sensor) * target_model_yaw
      yaw_fusion = 0.8 * yaw_fusion + 0.2 * nav_yaw_rate
      
      # 6. 相位超前预测 (80ms Lead) - 补偿控制延迟
      yaw_accel_raw = (yaw_fusion - self.last_yaw) * self.freq
      yaw_accel = self.yaw_accel_filter.update(yaw_accel_raw)
      self.last_yaw = yaw_fusion
      yaw_final = yaw_fusion + yaw_accel * 0.08

      # 7. 发布消息 (对齐 locationd 期望坐标系)
      now_ns = time.monotonic_ns()
      
      # 发布陀螺仪消息 (Z轴/Yaw)
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 发布加速度计消息 (X轴/Long, Y轴/Lat)
      accel_msg = messaging.new_message('accelerometer', valid=True)
      # 重点：使用真实传感器测得的侧向加速度 clean_lat，以支持路面横坡补偿 (Roll Compensation)
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': [0.0, -float(clean_lat), -float(clean_accel)], 'status': 1}}
      self.pm.send('accelerometer', accel_msg)

      # 调试输出
      if self.frame % 1000 == 0:
        print(f"Soul-IMU: v={cs.vEgo:.1f}, yaw={yaw_final:.4f}, lat={clean_lat:.4f}, ratio={self.learned_steer_ratio:.2f}")

      self.frame += 1
      self.rk.keep_time()

if __name__ == "__main__":
  NativeIMUD().main_loop()