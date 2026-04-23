#!/usr/bin/env python3
import time

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog


# ======================================
# 简单低通滤波
# ======================================
class LowPass:
  def __init__(self, alpha=0.35):
    self.alpha = alpha
    self.x = 0.0
    self.initialized = False

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

  yaw_filter = LowPass(0.3)
  lon_filter = LowPass(0.3)
  lat_filter = LowPass(0.3)

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

      yaw = cs.yawRate
      lon = cs.aEgo
      lat = cs.vEgo * cs.yawRate

      # ======================================
      # 滤波
      # ======================================

      yaw = yaw_filter.update(yaw)
      lon = lon_filter.update(lon)
      lat = lat_filter.update(lat)

      # ======================================
      # 异常保护
      # ======================================

      if abs(yaw) > 5:
        yaw = 0

      if abs(lon) > 20:
        lon = 0

      if abs(lat) > 20:
        lat = 0

      # ======================================
      # Gyroscope
      # 注意：timestamp 必须紧靠 send() 之前采样，
      # locationd.py 会比较 sensor_time=msg.timestamp*1e-9
      # 与 t=logMonoTime*1e-9 的差，超过 100ms 即丢弃！
      # ======================================

      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {
        'timestamp': time.monotonic_ns(),  # 紧靠send前采样，与logMonoTime对齐
        'type': 16,  # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
        'sensor': 5, # SENSOR_GYRO_UNCALIBRATED
        'source': 0, # log.SensorEventData.SensorSource.android
        'gyroUncalibrated': {
          'v': [0.0, 0.0, float(yaw)],
          'status': 1
        }
      }

      pm.send('gyroscope', gyro_msg)

      # ======================================
      # Accelerometer
      # ======================================

      accel_msg = messaging.new_message('accelerometer', valid=True)
      accel_msg.accelerometer = {
        'timestamp': time.monotonic_ns(),  # 再次采样，确保精准
        'type': 1,   # SENSOR_TYPE_ACCELEROMETER
        'sensor': 1, # SENSOR_ACCELEROMETER
        'source': 0, # log.SensorEventData.SensorSource.android
        'acceleration': {
          'v': [float(lon), float(lat), 0.0],
          'status': 1
        }
      }

      pm.send('accelerometer', accel_msg)

      # ======================================
      # 周期性打印 IMU 数据（每50帧约1秒）
      # 用于调试各方向数据是否正确传入 locationd
      # ======================================
      if frame % PRINT_INTERVAL == 0:
        raw_yaw = cs.yawRate
        raw_lon = cs.aEgo
        raw_lat = cs.vEgo * cs.yawRate
        print(
          f"[IMU] vEgo={cs.vEgo:.2f}m/s  "
          f"Raw: yawRate={raw_yaw:.4f}rad/s  aEgo={raw_lon:.4f}m/s²  lat_a={raw_lat:.4f}m/s²  |  "
          f"Filtered: gyro_z={yaw:.4f}  accel_x={lon:.4f}  accel_y={lat:.4f}  |  "
          f"→ gyro[0,0,{yaw:.4f}]  accel[{lon:.4f},{lat:.4f},0]"
        )

    rk.keep_time()


if __name__ == "__main__":
  main()