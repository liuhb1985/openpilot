#!/usr/bin/env python3

class Tuning:

  # 以下这组数据，仅力矩模式有效。当开启了LateralTorqueCustom = 1时，以下所有参数都无效。
  LAT_SIGLIN_TABLE = [4.867, 1.09, 0.243] #仅siglin模式有效

  STEERING_ANGLE_OFFSET = 0.0  # 修复：调整转向角度偏移，改善过弯精度 (从0.5调整为0.0，减少直行偏移)

  # 转向控制参数优化，用于改善转弯时的内外弯和压线问题
  STEER_RATIO_MULTIPLIER = 1.0  # 转向比率乘数，用于调整转弯时的转向力度
  # 降低转向比率乘数以减少过弯时的过度响应
  STEER_RATIO_BP = [0.0, 2.0, 5.0, 8.0, 12.0, 20.0, 30.0]  # 弯道曲率断点 (1/米)
  STEER_RATIO_FP = [1.0, 1.02, 1.05, 1.08, 1.10, 1.12, 1.15]   # 对应的转向比率乘数

  # 左转和右转的补偿参数
  LEFT_TURN_COMPENSATION = 1.1   # 左转补偿因子
  RIGHT_TURN_COMPENSATION = 1.1  # 右转补偿因子


  #速度修正参数
  DASHSPEED_BP = [5, 15, 30, 60, 90, 120]       #BP是车速
  DASHSPEED_FP = [0.7, 0.8, 1.0, 1.0, 1.0, 1.0] #修正百分比

  # modified stock long control 原车long控制的速度平滑百分比设定, 例如下面40米以内，则加速率是原来的70%，减速率是原来的100%
  K_ACCEL_BP       = [40,  50,  60,  70,  80]  # meters BP是离前车距离

  K_ACCEL_POS_4BAR = [0.8, 0.8, 0.8, 0.8, 0.8] # acceleration 加速的百分比
  K_ACCEL_NEG_4BAR = [0.9, 0.8, 0.7, 0.7, 0.7] # deceleration 减速的百分比

  K_ACCEL_POS_3BAR = [0.8, 0.8, 0.8, 0.8, 0.8] # acceleration 加速的百分比
  K_ACCEL_NEG_3BAR = [0.9, 0.85, 0.8, 0.75, 0.7] # deceleration 减速的百分比

  K_ACCEL_POS_2BAR = [0.9, 0.85, 0.8, 0.75, 0.7] # acceleration 加速的百分比
  K_ACCEL_NEG_2BAR = [1.0, 0.95, 0.9, 0.85, 0.8] # deceleration 减速的百分比

  K_ACCEL_POS_1BAR = [1.0, 0.95, 0.9, 0.85, 0.8] # acceleration 加速的百分比
  K_ACCEL_NEG_1BAR = [1.1, 1.05, 1.0, 0.95, 0.9] # deceleration 减速的百分比

  # 人为扭动方向盘的阈值，大于这个值才认为方向盘被故意扭动了，变道辅助涉及它
  STEER_PRESSED_THRESHOLD = 56

  # 禁用EPS故障检查, 某些车有EPS固件比较奇怪报错的话，则可以设为True
  DISABLE_EPS_WARNING = False
  DISABLE_EPS_TEMPORARY_FAULT = False
  DISABLE_EPS_PERMANENT_FAULT = False

  DISABLE_PARKBRAKE = False