#!/usr/bin/env python3
import time
import math
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

# ======================================
# V6 全知全能版 Native IMU 处理器 (终极进化型)
# ======================================

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'carParams', 'carrotMan'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    # 核心参数
    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)

    # 状态估计器 (Kalman)
    self.x = 0.0      # Yaw Rate
    self.P = 1.0
    self.Q = 0.0005   # 增加模型信任度
    self.R = 0.02
    
    # [V6 核心] 在线转向比学习 (RLS 估算器)
    self.learned_steer_ratio = 15.0
    self.rls_P = 1.0
    self.rls_lambda = 0.999 # 遗忘因子，保持长期稳定学习

    self.yaw_bias = 0.0
    self.bias_count = 0
    self.last_yaw = 0.0
    self.yaw_accel = 0.0
    self.model_yaw_lag = 0.0
    self.has_valid_data = False

  def update_calibration(self, cs):
    if cs.vEgo < 0.05 and not cs.steeringPressed:
      raw_yaw = math.radians(cs.yawRate)
      self.yaw_bias = (self.yaw_bias * self.bias_count + raw_yaw) / (self.bias_count + 1)
      self.bias_count = min(self.bias_count + 1, 1000)

  def learn_parameters(self, cs, raw_yaw):
    # 在转弯且速度稳定时，学习实际转向比
    v_ego = cs.vEgo
    steer_angle = math.radians(cs.steeringAngleDeg)
    if abs(v_ego) > 10.0 and abs(steer_angle) > 0.05 and abs(raw_yaw) > 0.02:
        # 物理公式变形: steerRatio = (v * tan(steer)) / (yaw * L)
        instant_ratio = (v_ego * math.tan(steer_angle)) / (raw_yaw * 2.8) # 2.8 轴距
        if 5.0 < instant_ratio < 25.0:
            # RLS 更新
            error = instant_ratio - self.learned_steer_ratio
            self.rls_P = self.rls_P / self.rls_lambda
            gain = self.rls_P / (1 + self.rls_P)
            self.learned_steer_ratio += gain * error
            self.rls_P = (1 - gain) * self.rls_P

  def main_loop(self):
    print("******** Omniscient IMU V6 Started (Global Fusion) ********")

    while True:
      self.sm.update(0)

      if self.sm.updated['carState']:
        cs = self.sm['carState']
        v_ego = cs.vEgo
        steer_angle = cs.steeringAngleDeg
        
        self.update_calibration(cs)

        # 1. 信号预处理
        raw_yaw = math.radians(cs.yawRate) - self.yaw_bias
        
        # 2. [V6] 在线参数学习
        self.learn_parameters(cs, raw_yaw)

        # 3. [V6] 引入导航先验曲率 ( carrotMan 提供的当前路径曲率 )
        nav_curvature = 0.0
        if self.sm.updated['carrotMan']:
            nav_curvature = self.sm['carrotMan'].atcCurvature # 假设 carrotMan 输出曲率
        nav_yaw_rate = v_ego * nav_curvature

        # 4. 物理模型预测 (使用学习到的转向比)
        steer_factor = 1.0 + (abs(steer_angle) * 0.0005)
        model_ratio = max(5.0, self.learned_steer_ratio * steer_factor) # 保护转向比
        if abs(v_ego) > 0.3:
          target_model_yaw = (v_ego * math.tan(math.radians(steer_angle) / model_ratio)) / 2.8
        else:
          target_model_yaw = 0.0
        
        # 混合导航前馈：让物理模型提前感知弯道
        model_yaw_combined = 0.8 * target_model_yaw + 0.2 * nav_yaw_rate
        self.model_yaw_lag = 0.2 * model_yaw_combined + 0.8 * self.model_yaw_lag

        # 5. 卡尔曼状态估计
        self.P = self.P + self.Q
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (raw_yaw - self.x)
        self.P = (1 - K) * self.P
        
        # 6. 极致稳定融合
        blend_factor = np.interp(abs(steer_angle), [0.5, 2.0], [0.1, 0.5])
        yaw = blend_factor * self.x + (1.0 - blend_factor) * self.model_yaw_lag

        # 7. 全维相位补偿预测
        yaw_accel = (yaw - self.last_yaw) / self.dt
        yaw_final = yaw + yaw_accel * 0.03 # 30ms 深度预测

        # 8. 加速度处理
        lat = v_ego * yaw_final
        
        # 数据合理性检查
        if not np.isfinite(yaw_final) or abs(yaw_final) > 2.0:
            yaw_final = 0.0
        if not np.isfinite(lat) or abs(lat) > 20.0:
            lat = 0.0

        self.last_yaw, self.last_lat = yaw, lat
        # 发布消息
        now_ns = time.monotonic_ns()
        gyro_msg = messaging.new_message('gyroscope', valid=True)
        gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                             'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}}
        self.pm.send('gyroscope', gyro_msg)

        accel_msg = messaging.new_message('accelerometer', valid=True)
        accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                                 'acceleration': {'v': [0.0, -float(lat), -float(cs.aEgo)], 'status': 1}}
        self.pm.send('accelerometer', accel_msg)

        if self.rk.frame % 100 == 0:
          print(f"IMU: v={v_ego:.1f}, yaw={yaw_final:.4f}, lat={lat:.3f}, ratio={self.learned_steer_ratio:.2f}, nav_curv={nav_curvature:.5f}")

      self.rk.keep_time()

def main():
  imud = NativeIMUD()
  imud.main_loop()

if __name__ == "__main__":
  main()