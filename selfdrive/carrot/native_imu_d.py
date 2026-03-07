#!/usr/bin/env python3
import time
import numpy as np
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params

def main():
  params = Params()
  # 只有在非 JY62 模式下才建议运行此服务，或者由 manager 根据配置启动

  sm = messaging.SubMaster(['carState'])
  pm = messaging.PubMaster(['gyroscope', 'accelerometer'])
  rk = Ratekeeper(100, print_delay_threshold=None) # 100Hz 匹配现代底盘频率

  print("************************************************Native IMU Service Started************************************************")

  while True:
    sm.update(0)

    if sm.updated['carState']:
      cs = sm['carState']

      # 创建陀螺仪消息 (Angular Velocity)
      # openpilot 坐标系: x=前, y=左, z=上 (基于 LSM6DS3 映射)
      gyro_msg = messaging.new_message('gyroscope')
      gyro_msg.valid = True

      g = gyro_msg.gyroscope.gyroUncalibrated
      g.timestamp = int(time.monotonic() * 1e9)
      g.v = [0.0, 0.0, cs.yawRate]
      g.status = 1

      pm.send('gyroscope', gyro_msg)


      accel_msg = messaging.new_message('accelerometer')
      accel_msg.valid = True

      a = accel_msg.accelerometer.acceleration
      a.timestamp = int(time.monotonic() * 1e9)
      a.v = [cs.longitudinalAcceleration, cs.lateralAcceleration, 9.80665]
      a.status = 1

      pm.send('accelerometer', accel_msg)

    rk.keep_time()

if __name__ == "__main__":
  main()
