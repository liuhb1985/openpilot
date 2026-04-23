#!/usr/bin/env python3
import time
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

# ======================================
# 坐标系说明
# ======================================
# locationd.py 的坐标变换规则（Comma设备→车辆坐标系）：
#   gyro:  meas = [-v[2], -v[1], -v[0]]
#   accel: meas = [-v[2], -v[1], -v[0]]
#
# PoseKalman 期望的车辆坐标系：
#   meas[0] = X = 向前（纵向）/ roll
#   meas[1] = Y = 向左（侧向）/ pitch
#   meas[2] = Z = 向上（垂直）/ yaw
#
# 要让 yaw_rate → meas[2]：需要 v[0] = -yaw_rate → meas[2] = -v[0] = yaw_rate ✓
# 要让 lon_accel → meas[0]：需要 v[2] = -lon     → meas[0] = -v[2] = lon     ✓
# 要让 lat_accel → meas[1]：需要 v[1] = -lat     → meas[1] = -v[1] = lat     ✓

# ======================================
# 简单低通滤波
# ======================================
class LowPass:
  def __init__(self, alpha=0.25):
    self.alpha = alpha
    self.x = 0.0
    self.initialized = False

  def reset(self, v):
    self.x = v
    self.initialized = True

  def update(self, v):
    if not self.initialized:
      self.x = v
      self.initialized = True
    else:
      self.x = self.alpha * v + (1 - self.alpha) * self.x
    return self.x


# ======================================
# 主程序
# ======================================
def main():

  sm = messaging.SubMaster(['carState'])
  pm = messaging.PubMaster(['gyroscope', 'accelerometer'])

  rk = Ratekeeper(50, print_delay_threshold=None)

  # [优化] yaw 用更慢的 alpha（信号更稳），lon/lat 用 0.25
  yaw_filter = LowPass(0.2)
  lon_filter = LowPass(0.25)
  lat_filter = LowPass(0.25)

  # [优化] 保活：记录上一帧的有效值，carState 未更新时继续发布
  last_yaw = 0.0
  last_lon = 0.0
  last_lat = 0.0
  has_valid_data = False

  print("******** Native Vehicle IMU Started ********")

  frame = 0
  PRINT_INTERVAL = 50  # 每50帧打印一次（约1秒）

  while True:

    sm.update(0)
    frame += 1

    if sm.updated['carState']:

      cs = sm['carState']

      # ======================================
      # 原车 IMU 数据
      # ======================================

      raw_yaw = cs.yawRate            # 横摆角速度 (rad/s)
      raw_lon = cs.aEgo               # 纵向加速度 (m/s²)

      # [优化] 侧向加速度：低速时线性淡出，避免低速 yawRate 噪声被 vEgo 放大
      # 在 vEgo < 1.0 m/s 时直接用 0，1.0~3.0 m/s 线性插值
      v = cs.vEgo
      lat_scale = float(np.interp(abs(v), [1.0, 3.0], [0.0, 1.0]))
      raw_lat = v * cs.yawRate * lat_scale

      # ======================================
      # [修复] 先做异常保护，再滤波（防止异常值污染滤波器状态）
      # ======================================

      if abs(raw_yaw) > 5.0:
        raw_yaw = last_yaw  # 用上一帧值替代，避免滤波器跳变
      if abs(raw_lon) > 20.0:
        raw_lon = last_lon
      if abs(raw_lat) > 20.0:
        raw_lat = last_lat

      # ======================================
      # 滤波
      # ======================================

      yaw = yaw_filter.update(raw_yaw)
      lon = lon_filter.update(raw_lon)
      lat = lat_filter.update(raw_lat)

      last_yaw = yaw
      last_lon = lon
      last_lat = lat
      has_valid_data = True

    elif has_valid_data:
      # [优化] carState 未更新时使用上一帧的滤波值发布（保活，防止 sensor_alive 超时）
      yaw = last_yaw
      lon = last_lon
      lat = last_lat
    else:
      # 还没有任何有效数据，跳过发布
      rk.keep_time()
      continue

    # ======================================
    # [修复] 在两次 send 之前统一采样一次时间戳，保证 gyro/accel 时间戳一致
    # locationd 会比较 sensor_time=msg.timestamp*1e-9 与 t=logMonoTime*1e-9 的差
    # 超过 100ms 即丢弃。两者必须尽量靠近 pm.send() 调用时刻。
    # ======================================

    now_ns = time.monotonic_ns()

    # ======================================
    # Gyroscope
    # locationd 变换: meas = [-v[2], -v[1], -v[0]]
    # 目标: meas[2] = yaw_rate（Z轴=横摆）
    # 因此: v[0] = -yaw → meas[2] = -v[0] = yaw ✓
    # ======================================

    gyro_msg = messaging.new_message('gyroscope', valid=True)
    gyro_msg.gyroscope = {
      'timestamp': now_ns,           # [修复] 统一时间戳
      'type': 16,  # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
      'sensor': 5, # SENSOR_GYRO_UNCALIBRATED
      'source': 0, # log.SensorEventData.SensorSource.android
      'gyroUncalibrated': {
        # v[0]=-yaw → meas[2]=-v[0]=yaw (Z轴=横摆) ✓
        # v[1]=0    → meas[1]=-v[1]=0  (Y轴=俯仰) ✓
        # v[2]=0    → meas[0]=-v[2]=0  (X轴=横滚) ✓
        'v': [-float(yaw), 0.0, 0.0],
        'status': 1
      }
    }

    pm.send('gyroscope', gyro_msg)

    # ======================================
    # Accelerometer
    # locationd 变换: meas = [-v[2], -v[1], -v[0]]
    # 目标:
    #   meas[0] = lon (X轴=向前纵向)
    #   meas[1] = lat (Y轴=向左侧向)
    #   meas[2] = 0   (Z轴=垂直，忽略重力)
    # 因此:
    #   v[2] = -lon → meas[0] = -v[2] = lon ✓
    #   v[1] = -lat → meas[1] = -v[1] = lat ✓
    #   v[0] = 0    → meas[2] = -v[0] = 0   ✓
    # ======================================

    accel_msg = messaging.new_message('accelerometer', valid=True)
    accel_msg.accelerometer = {
      'timestamp': now_ns,           # [修复] 统一时间戳，与 gyro 完全一致
      'type': 1,   # SENSOR_TYPE_ACCELEROMETER
      'sensor': 1, # SENSOR_ACCELEROMETER
      'source': 0, # log.SensorEventData.SensorSource.android
      'acceleration': {
        # v[0]=0    → meas[2]=-v[0]=0   (Z轴=垂直) ✓
        # v[1]=-lat → meas[1]=-v[1]=lat (Y轴=侧向) ✓
        # v[2]=-lon → meas[0]=-v[2]=lon (X轴=纵向) ✓
        'v': [0.0, -float(lat), -float(lon)],
        'status': 1
      }
    }

    pm.send('accelerometer', accel_msg)

    # ======================================
    # 周期性打印 IMU 数据（每50帧约1秒）
    # ======================================
    if frame % PRINT_INTERVAL == 0:
      # [修复] 直接使用本帧已处理的局部变量，避免在非carState更新帧时
      # sm.updated['carState']=False 但 sm['carState'] 仍返回旧值导致的信息混乱
      if has_valid_data:
        # 从 sm['carState'] 取 vEgo 仅用于显示（sm 保存最近一次有效值）
        v_display = sm['carState'].vEgo
      else:
        v_display = -1.0
      print(
        f"[IMU] vEgo={v_display:.2f}m/s  "
        f"Filtered: yaw={yaw:.4f}rad/s  lon={lon:.4f}m/s²  lat={lat:.4f}m/s²  |  "
        f"gyro_v[{-yaw:.4f},0,0]  accel_v[0,{-lat:.4f},{-lon:.4f}]"
      )

    rk.keep_time()


if __name__ == "__main__":
  main()