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
    self.yaw_bias = None
    self.lat_bias = None
    self.bias_count = 0
    self.bias_stabled = False
    
    # 自适应滤波器集
    # [修复] CAN 总线 yawRate 有严重的量化噪声，需要更强的滤波
    # 时间常数从 0.05s 提升到 0.3s，平滑掉 CAN 量化跳变
    self.yaw_filter = FirstOrderFilter(0.0, 0.3, self.dt)
    self.accel_filter = FirstOrderFilter(0.0, 0.1, self.dt)
    self.yaw_accel_filter = FirstOrderFilter(0.0, 0.2, self.dt)
    
    self.last_yaw = 0.0
    
    # 转向比学习状态
    try:
      sr_val = self.params.get("LatMpcSteerRatio")
      sr = float(sr_val) if sr_val else 15.0
    except:
      sr = 15.0
    self.learned_steer_ratio = max(13.0, min(20.0, sr))
    self.rls_P = 1.0
    
    # RLS 动态记忆因子
    self.base_lambda = 0.9998
    self.fast_lambda = 0.995

  def update_calibration(self, cs):
    # 初始化偏置 (仅在静止且无转向输入时初始化，防止捕获行驶状态)
    if self.yaw_bias is None and cs.vEgo < 0.1 and not cs.steeringPressed: 
      self.yaw_bias = cs.yawRate
      print(f"[IMU_INIT] Initialized Yaw Bias: {self.yaw_bias:.5f}")
    if self.lat_bias is None and cs.vEgo < 0.1 and not cs.steeringPressed: 
      self.lat_bias = cs.latAccel
      print(f"[IMU_INIT] Initialized Lat Bias: {self.lat_bias:.5f}")

    # 1. 静态校准 (车速接近0且方向盘无输入)
    if cs.vEgo < 0.01 and not cs.steeringPressed:
      self.bias_count += 1
      # 增加动态收敛速率
      alpha = 1.0 / min(self.bias_count, 1000)
      self.yaw_bias = (1 - alpha) * self.yaw_bias + alpha * cs.yawRate
      self.lat_bias = (1 - alpha) * self.lat_bias + alpha * cs.latAccel
      if self.bias_count > 500: self.bias_stabled = True
    
    # 2. 动态行驶校准 (高速直行时自动修正 Bias)
    elif cs.vEgo > 10.0 and abs(cs.steeringAngleDeg) < 5.0:
      # [修复] 降低动态校准学习率，防止 bias 追随噪声跳动
      self.yaw_bias = 0.999 * self.yaw_bias + 0.001 * cs.yawRate
      if self.frame % 100 == 0:
        print(f"[IMU_CALIB] Adjusting Bias: {self.yaw_bias:.5f}, RawYaw: {cs.yawRate:.5f}")

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
    print("******** Native IMU V7.2 Baseline Restored ********")
    
    while True:
      self.sm.update(0)
      if not self.sm.updated['carState']:
        self.rk.keep_time()
        continue
        
      cs = self.sm['carState']
      v_ego = cs.vEgo
      self.update_calibration(cs)
      
      # 1. 信号平滑处理 (保持 100Hz 高频响应)
      clean_yaw = self.yaw_filter.update(cs.yawRate - self.yaw_bias)
      
      # [深度优化] 物理一致性合成逻辑
      # 如果 cs.latAccel 缺失 (为0)，根据动力学公式 latAccel = vEgo * yawRate 合成
      # 这能解决 locationd 因 yawRate 有波动但 latAccel 为 0 导致的卡尔曼滤波震荡（R线波动）
      if abs(cs.latAccel) < 0.001 and v_ego > 1.0:
        raw_lat = v_ego * clean_yaw
      else:
        raw_lat = cs.latAccel - self.lat_bias
      clean_lat = self.accel_filter.update(raw_lat)
      
      # 2. 转向比自适应演进 (保留核心学习逻辑)
      angle_offset = self.sm['liveParameters'].angleOffsetDeg if self.sm.seen['liveParameters'] else 0.0
      steer_angle_rad = math.radians(cs.steeringAngleDeg - angle_offset)
      self.learn_steer_ratio(v_ego, steer_angle_rad, clean_yaw)
      
      # 3. 动态融合 (回归简洁：仅在极低速下使用模型辅助)
      model_weight = np.interp(v_ego, [0.0, 1.0], [1.0, 0.0])
      wheelbase = self.sm['carParams'].wheelbase if self.sm.seen['carParams'] else 2.8
      model_yaw = (v_ego * math.tan(steer_angle_rad / self.learned_steer_ratio)) / wheelbase
      
      yaw_fusion = model_weight * model_yaw + (1.0 - model_weight) * clean_yaw
      
      # 4. 移除相位预测 (不再使用 yaw_accel * 0.04)，确保 locationd 校验通过
      yaw_final = np.clip(yaw_fusion, -2.0, 2.0)
      
      # 5. 发布经过物理映射的传感器数据 (满足 locationd 坐标系要求)
      now_ns = time.monotonic_ns()
      
      # 陀螺仪: 映射后满足 locationd 变换 meas = [-v[2], -v[1], -v[0]] -> meas[2] = -v[0] = +yaw_final (Z-up)
      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {'timestamp': now_ns, 'type': 16, 'sensor': 5, 
                           'gyroUncalibrated': {'v': [-float(yaw_final), 0.0, 0.0], 'status': 1}}
      self.pm.send('gyroscope', gyro_msg)

      # 加速度计: 映射后满足 meas[0] = lon, meas[1] = lat (Left is positive), meas[2] = gravity
      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {'timestamp': now_ns, 'type': 1, 'sensor': 1, 
                               'acceleration': {'v': [G, -float(clean_lat), -float(cs.aEgo)], 'status': 1}}
      self.pm.send('accelerometer', accel_msg)

      if self.frame % 1000 == 0:
        status = "Calibrating" if not self.bias_stabled else "Running"
        print(f"[{status}] v={v_ego:.1f}, SR={self.learned_steer_ratio:.2f}, Bias={self.yaw_bias:.5f}")

      self.frame += 1
      self.rk.keep_time()

if __name__ == "__main__":
  main()