#!/usr/bin/env python3
import math
import time
import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter

def main():
  NativeIMUD().main_loop()

# 物理常数
G = 9.80665

class NativeIMUD:
  def __init__(self):
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'carParams', 'liveParameters', 'carrotMan'])
    self.pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

    self.freq = 100.0
    self.dt = 1.0 / self.freq
    self.rk = Ratekeeper(self.freq, print_delay_threshold=None)
    self.frame = 0
    
    # 传感器偏置校准
    self.yaw_bias = 0.0
    self.lat_bias = 0.0
    self.bias_count = 0
    self.bias_stabled = False
    
    # 自适应滤波器集
    self.yaw_filter = FirstOrderFilter(0.0, 0.05, self.dt)
    self.accel_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.yaw_accel_filter = FirstOrderFilter(0.0, 0.2, self.dt)
    
    self.last_yaw = 0.0
    
    # 转向比学习状态
    sr = self.params.get_float("LatMpcSteerRatio") if self.params.get("LatMpcSteerRatio") else 15.0
    self.learned_steer_ratio = max(13.0, min(20.0, sr))
    self.rls_P = 1.0
    
    # RLS 动态记忆因子
    self.base_lambda = 0.9998
    self.fast_lambda = 0.995

  def update_calibration(self, cs):
    # 只有在车速接近0且方向盘无输入时校准
    if cs.vEgo < 0.01 and not cs.steeringPressed:
      self.bias_count += 1
      # 增加动态收敛速率
      alpha = 1.0 / min(self.bias_count, 1000)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * cs.yawRate
      self.lat_bias = (1 - alpha) * self.lat_bias + alpha * cs.latAccel
      if self.bias_count > 500: self.bias_stabled = True

  def learn_steer_ratio(self, v_ego, steer_angle_rad, clean_yaw):
    wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
    
    # 严苛的学习触发条件：速度 > 36km/h，转向角 > 3度，YawRate 显著
    if v_ego > 10.0 and abs(steer_angle_rad) > 0.052 and abs(clean_yaw) > 0.01:
      # 逆自行车模型计算
      denominator = math.atan2(clean_yaw * wheelbase, v_ego)
      if abs(denominator) > 1e-4:
        instant_ratio = steer_angle_rad / denominator
        
        if 11.0 < instant_ratio < 22.0:
          error = instant_ratio - self.learned_steer_ratio
          
          # 根据误差大小动态调整学习率 (Error-Aware Lambda)
          curr_lambda = self.fast_lambda if abs(error) > 0.5 else self.base_lambda
          
          self.rls_P = self.rls_P / curr_lambda
          gain = self.rls_P / (1 + self.rls_P)
          self.learned_steer_ratio += gain * error
          self.rls_P = (1 - gain) * self.rls_P
          self.learned_steer_ratio = np.clip(self.learned_steer_ratio, 13.0, 20.0)

  def main_loop(self):
    print("******** Native IMU V7.1 Pro Optimizer ********")
    
    while True:
      self.sm.update(0)
      if not self.sm.updated['carState']:
        self.rk.keep_time()
        continue
        
      cs = self.sm['carState']
      v_ego = cs.vEgo
      self.update_calibration(cs)
      
      # 1. 信号平滑处理 (自适应响应)
      # 高速时降低平滑(T=0.03)，增加灵敏度；低速时增加平滑(T=0.1)
      yaw_t = np.interp(v_ego, [0.0, 20.0], [0.1, 0.03])
      self.yaw_filter.update_alpha(yaw_t, self.dt)
      
      clean_yaw = self.yaw_filter.update(cs.yawRate - self.yaw_bias)
      clean_lat = self.accel_filter.update(cs.latAccel - self.lat_bias)
      
      # 2. 转向比自适应演进
      angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
      steer_angle_rad = math.radians(cs.steeringAngleDeg - angle_offset)
      self.learn_steer_ratio(v_ego, steer_angle_rad, clean_yaw)
      
      # 3. 动态多源融合 (Speed-Weighted Fusion)
      # 在超低速(<3.6km/h)下，传感器信噪比极低，此时 100% 信任模型预测
      model_weight = np.interp(v_ego, [0.5, 2.0], [1.0, 0.0])
      wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
      model_yaw = (v_ego * math.tan(steer_angle_rad / self.learned_steer_ratio)) / wheelbase
      
      yaw_fusion = model_weight * model_yaw + (1.0 - model_weight) * clean_yaw
      
      # 4. 相位预测 (40ms Lead)
      yaw_accel_raw = (yaw_fusion - self.last_yaw) * self.freq
      yaw_accel = self.yaw_accel_filter.update(yaw_accel_raw)
      self.last_yaw = yaw_fusion
      yaw_final = yaw_fusion + yaw_accel * 0.04
      
      # 5. 发布经过物理映射的传感器数据
      now_ns = time.monotonic_ns()
      
      # 陀螺仪: 映射后满足 locationd 变换 meas = [-v[2], -v[1], -v[0]] -> meas[2] = yaw
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 加速度计: 映射后满足 meas[0] = lon, meas[1] = lat
      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': [0.0, float(clean_lat), -float(cs.aEgo)], 'status': 1}}
      self.pm.send('accelerometer', accel_msg)

      if self.frame % 1000 == 0:
        status = "Calibrating" if not self.bias_stabled else "Running"
        print(f"[{status}] v={v_ego:.1f}, SR={self.learned_steer_ratio:.2f}, Bias={self.yaw_bias:.5f}")

      self.frame += 1
      self.rk.keep_time()

def main():
  NativeIMUD().main_loop()

if __name__ == "__main__":
  main()