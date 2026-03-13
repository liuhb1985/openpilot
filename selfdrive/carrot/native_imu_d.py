#!/usr/bin/env python3
import time

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper


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

  while True:

    sm.update(0)

    if sm.updated['carState']:

      cs = sm['carState']

      timestamp = int(time.monotonic() * 1e9)

      # ======================================
      # 原车 IMU 数据
      # ======================================

      yaw = cs.yawRate
      lon = cs.longitudinalAcceleration
      lat = cs.lateralAcceleration

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
      # ======================================

      gyro_msg = messaging.new_message('gyroscope', valid=True)
      gyro_msg.gyroscope = {
        'timestamp': timestamp,
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
        'timestamp': timestamp,
        'type': 1,   # SENSOR_TYPE_ACCELEROMETER
        'sensor': 1, # SENSOR_ACCELEROMETER
        'source': 0, # log.SensorEventData.SensorSource.android
        'acceleration': {
          'v': [float(lon), float(lat), 0.0],
          'status': 1
        }
      }

      pm.send('accelerometer', accel_msg)

    rk.keep_time()


if __name__ == "__main__":
  main()