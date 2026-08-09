import numpy as np
import math
import time
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd.tuning import Tuning

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)

    self.packer = CANPacker(dbc_names[Bus.pt])
    from cereal import messaging
    self.sm = messaging.SubMaster(['radarState', 'modelV2', 'longitudinalPlan'])
    self.frame = 0
    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0
    self.lat_safeoff = 0

    self.steer_softstart_limit = 0
    self.steerRateLimActive = False
    self.steerRateLim = 1.0

    self.first_start = True
    self.rfss = 0 # resume from stand still
    self.sss = 0 #stand still state

    self.apply_accel_last = 0

    # 用于检测前车距离变化（判断前车是否移动）
    self.last_lead_distance = 0.0
    self.last_update_time = 0.0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # 横向控制部分 - 保持原有逻辑
    if (self.frame - self.last_steer_frame) >= CarControllerParams.STEER_STEP:
      if self.first_start:
        self.mpc_lkas_counter = int(CS.acc_mpc_state_counter + 1) & 0xF
        self.mpc_acc_counter = int(CS.acc_cmd_counter + 1) & 0xF
        self.eps_fake318_counter = int(CS.eps_state_counter + 1) & 0xF
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        if self.lkas_active:
          steer_desire = CC.actuators.torque

          if CarControllerParams.USE_STEERING_SPEED_LIMITER:
            # 获取转弯曲率，用于调整转向力度
            curvature = abs(CS.out.yawRate / max(CS.out.vEgo, 0.1))  # 曲率 = 偏航率 / 速度

            # 根据转弯曲率调整转向比率
            def interp(x, xp, fp):
                if x <= xp[0]:
                    return fp[0]
                if x >= xp[-1]:
                    return fp[-1]
                for i in range(len(xp) - 1):
                    if xp[i] <= x <= xp[i + 1]:
                        return fp[i] + (fp[i + 1] - fp[i]) * (x - xp[i]) / (xp[i + 1] - xp[i])
                return fp[0]

            steer_ratio_multiplier = interp(curvature, CarControllerParams.STEER_RATIO_BP, CarControllerParams.STEER_RATIO_FP)

            # 根据左转或右转应用不同的补偿因子
            if CS.out.yawRate > 0.01:  # 左转
              steer_ratio_multiplier *= CarControllerParams.LEFT_TURN_COMPENSATION
            elif CS.out.yawRate < -0.01:  # 右转
              steer_ratio_multiplier *= CarControllerParams.RIGHT_TURN_COMPENSATION

            # 优化：针对BYD Han在有前车或地平线较短时的转向速率限制
            # 低速时更严格的限制，防止摆动
            speed_kph = CS.out.vEgo * 3.6
            # 获取前车距离数据，优先使用雷达融合数据，其次使用BYD雷达数据
            if hasattr(self, 'sm') and self.sm.alive['radarState']:
                lead_one = self.sm['radarState'].leadOne
                if lead_one.status:
                    lead_distance = lead_one.dRel if not math.isnan(lead_one.dRel) else 199
                else:
                    lead_distance = getattr(CS, 'mrr_leading_dist', 199)
            else:
                lead_distance = getattr(CS, 'mrr_leading_dist', 199)

            # 只有在前车距离30米以内时才应用特殊的转向速率限制
            if lead_distance < 30:
                # 获取车道曲率信息，用于判断是否在过弯
                model_curvature = 0.0
                if hasattr(self, 'sm') and self.sm.alive['modelV2']:
                    model = self.sm['modelV2']
                    if hasattr(model, 'orientationRate') and len(model.orientationRate.z) > 0:
                        # 使用偏航率作为曲率的近似值
                        model_curvature = abs(model.orientationRate.z[0]) if not math.isnan(model.orientationRate.z[0]) else 0.0

                # 根据车速、前车距离和车道曲率动态调整转向速率限制
                # 只有在前车距离较近(30米以内)且不在急弯道上时才应用严格的转向速率限制
                if speed_kph < 20 and model_curvature < 0.05:  # 低速且直线行驶
                  # 进一步降低转向速率限制，防止在跟车时摆动
                  rate_limit = np.interp(speed_kph, [0, 10, 20], [8, 15, 30])
                elif speed_kph < 20 and model_curvature < 0.1:  # 低速且轻微弯道
                  # 中等严格的限制
                  rate_limit = np.interp(speed_kph, [0, 10, 20], [12, 25, 45])
                elif speed_kph < 20:  # 低速但在弯道上
                  # 相对宽松的限制，确保过弯能力
                  rate_limit = np.interp(speed_kph, [0, 10, 20], [20, 40, 65])
                elif speed_kph < 40 and model_curvature < 0.05:  # 中速且直线行驶
                  # 严格的限制
                  rate_limit = np.interp(speed_kph, [20, 30, 40], [30, 40, 55])
                elif speed_kph < 40 and model_curvature < 0.1:  # 中速且轻微弯道
                  # 中等限制
                  rate_limit = np.interp(speed_kph, [20, 30, 40], [40, 55, 70])
                elif speed_kph < 40:  # 中速但在弯道上
                  # 相对宽松的限制
                  rate_limit = np.interp(speed_kph, [20, 30, 40], [55, 70, 85])
                else:  # 高速或在急弯道上
                  # 最宽松的限制，确保高速过弯能力
                  rate_limit = np.interp(speed_kph, [40, 60, 80], [70, 80, 90])
            else:
                # 前车距离超过30米，使用原来的逻辑
                if speed_kph < 20:  # 低速时（<20km/h）
                  rate_limit = np.interp(speed_kph, [0, 10, 20], [30, 50, 80])
                else:  # 正常速度
                  rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [80, 50])

            delta_rate = CS.steeringRateDegAbs - rate_limit

            if delta_rate < 0:
              self.steerRateLim -= 0.003 * delta_rate  # 减慢恢复速度
              if delta_rate < -0.05:
                self.steerRateLimActive = False
              if self.steerRateLim > 1.0:
                self.steerRateLim = 1.0
                self.steerRateLimActive = False
            else:
              if self.steerRateLimActive:
                self.steerRateLim -= 0.008 * delta_rate  # 加快限制响应
              else:
                self.steerRateLim = steer_desire
                self.steerRateLimActive = True
              if self.steerRateLim < 0:
                self.steerRateLim = 0

            new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
            
            # 应用转弯优化的转向比率乘数
            new_steer_pu *= steer_ratio_multiplier
            
            # 应用转向角度偏移
            new_steer_pu += CarControllerParams.STEERING_ANGLE_OFFSET
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * CarControllerParams.STEER_MAX))

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
            self.steer_softstart_limit = self.steer_softstart_limit + CarControllerParams.STEER_SOFTSTART_STEP
            new_steer = np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          apply_torque = apply_driver_steer_torque_limits(new_steer, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
        else:
          if CS.lkas_prepared:
            self.lkas_active = 1.0
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
            self.lat_safeoff = 1
          else:
            self.lkas_req_prepare = 1

      elif self.lat_safeoff:
        if self.apply_torque_last == 0:
          self.lat_safeoff = 0
        apply_torque = apply_driver_steer_torque_limits(0, self.apply_torque_last,
                                                          CS.out.steeringTorque, CarControllerParams)
      else:
        self.lkas_req_prepare = 0
        self.steerRateLimActive = False
        self.steerRateLim = 1.0
        self.lkas_active = 0
        self.steer_softstart_limit = 0

      self.apply_torque_last = apply_torque

      self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
      self.eps_fake318_counter = int(self.eps_fake318_counter + 1) & 0xF
      self.last_steer_frame = self.frame

      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_torque_last, self.lkas_req_prepare, self.lkas_active, CC.hudControl, self.mpc_lkas_counter))

      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                              CS.mpc_laks_output, CS.mpc_laks_reqprepare, CS.mpc_laks_active,
                                              True, self.eps_fake318_counter))

    # 纵向控制部分 - 信任MPC输出，只做安全限制
    if (self.frame + 1 - self.last_acc_frame) >= CarControllerParams.ACC_STEP:
      # 更新雷达数据
      self.sm.update(0)

      mpc_target_accel = CC.actuators.accel

      if CC.longActive:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        running = CC.actuators.longControlState == LongCtrlState.pid

        # 获取基本数据用于日志记录（不用于控制逻辑）
        lead_distance = getattr(CS, 'mrr_leading_dist', 199)
        v_ego = CS.out.vEgo

        # 获取雷达融合数据用于日志
        lead_speed = 0.0
        relative_speed = 0.0
        fusion_distance = 199
        data_source = "no_radar"

        # 获取视觉前车距离（用于判断前车是否移动）
        if hasattr(self, 'sm') and self.sm.alive['modelV2']:
            lead_data = self.sm['modelV2'].lead
            if lead_data.prob > 0.5 and not math.isnan(lead_data.x[0]):
                vision_lead_distance = lead_data.x[0]
                # 计算距离变化率（前车移动速度）
                if self.last_lead_distance > 0:
                    dt = time.time() - self.last_update_time
                    if dt > 0:
                        # 距离在减小，说明前车在移动
                        relative_speed = (self.last_lead_distance - vision_lead_distance) / dt
                self.last_lead_distance = vision_lead_distance
                self.last_update_time = time.time()

        if hasattr(self, 'sm') and self.sm.alive['radarState']:
            lead_one = self.sm['radarState'].leadOne
            if lead_one.status:
                lead_speed = lead_one.vLead if not math.isnan(lead_one.vLead) else 0.0
                fusion_distance = lead_one.dRel
                data_source = "radar"
            else:
                data_source = "no_lead"

        # 车辆特定的安全限制和平滑处理
        # 信任MPC的计算，只对极端情况进行安全限制
        if mpc_target_accel < 0:
            # 基于融合数据的动态制动缩放
            if fusion_distance < 199:
                # 距离因子：针对快速接近场景优化
                if relative_speed < -2.0 and fusion_distance < v_ego * 1.5:
                    # 快速接近时，增强制动响应
                    distance_factor = 1.0  # 不缩放制动
                    speed_factor = 1.2     # 增强制动
                else:
                    # 正常情况的缩放
                    distance_factor = np.interp(fusion_distance, [5.0, 30.0], [0.8, 0.4])
                    if relative_speed < -1.0:
                        speed_factor = 1.0
                    elif relative_speed < 0:
                        speed_factor = 0.7
                    else:
                        speed_factor = 0.5

                # 速度因子：相对速度越大（接近前车），制动缩放越大
                if relative_speed < -1.0:  # 快速接近前车
                    speed_factor = 1.0
                elif relative_speed < 0:   # 缓慢接近前车
                    speed_factor = 0.7
                else:                      # 远离前车或速度匹配
                    speed_factor = 0.5

                # 综合缩放因子
                brake_scale = distance_factor * speed_factor
                brake_scale = np.clip(brake_scale, 0.3, 0.8)
            else:
                # 无前车时大幅减少制动
                brake_scale = 0.3

            # 应用tuning.py中的K_ACCEL参数进行进一步调整
            # 根据跟车距离选择对应的减速百分比
            if fusion_distance < 199:
                # 获取ACC档位（从CarState读取SetDistance: 1-4档）
                acc_bar = getattr(CS, 'adas_set_dist', 4)  # 默认4档（最远距离）

                # 根据档位选择对应的K_ACCEL参数
                if acc_bar == 4:
                    k_accel_neg = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_4BAR)
                elif acc_bar == 3:
                    k_accel_neg = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_3BAR)
                elif acc_bar == 2:
                    k_accel_neg = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_2BAR)
                else:  # acc_bar == 1
                    k_accel_neg = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_1BAR)

                # 应用K_ACCEL减速修正
                scaled_accel = mpc_target_accel * brake_scale * k_accel_neg
            else:
                scaled_accel = mpc_target_accel * brake_scale
        else:
            # 加速指令 - 应用tuning.py中的K_ACCEL参数
            if fusion_distance < 199:
                # 获取ACC档位（从CarState读取SetDistance: 1-4档）
                acc_bar = getattr(CS, 'adas_set_dist', 4)  # 默认4档（最远距离）

                # 根据档位选择对应的K_ACCEL参数
                if acc_bar == 4:
                    k_accel_pos = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_4BAR)
                elif acc_bar == 3:
                    k_accel_pos = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_3BAR)
                elif acc_bar == 2:
                    k_accel_pos = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_2BAR)
                else:  # acc_bar == 1
                    k_accel_pos = np.interp(fusion_distance, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_1BAR)

                # 应用K_ACCEL加速修正
                scaled_accel = mpc_target_accel * k_accel_pos
            else:
                # 无前车时直接使用MPC输出
                scaled_accel = mpc_target_accel

        # 平滑处理 - 防止加速度突变
        if hasattr(self, 'last_final_accel'):
            # 检测MPC的极端跳跃
            if hasattr(self, 'last_mpc_accel'):
                mpc_change = abs(mpc_target_accel - self.last_mpc_accel)
                accel_change_limit = 0.1 if mpc_change > 2.0 else 0.2
            else:
                accel_change_limit = 0.25

            accel_diff = scaled_accel - self.last_final_accel
            if abs(accel_diff) > accel_change_limit:
                scaled_accel = self.last_final_accel + np.sign(accel_diff) * accel_change_limit

        self.last_mpc_accel = mpc_target_accel
        final_accel = np.clip(scaled_accel, -2.5, 1.0)  # 合理的加速度限制
        self.last_final_accel = final_accel

        # 停车状态逻辑
        if stopping and final_accel < -0.1:
          self.rfss = 0
          self.sss = CS.out.standstill
        elif starting and final_accel > 0.1 and CS.out.vEgo < 0.8:
          self.rfss = CS.out.standstill
          self.sss = 0
        elif running:
          self.rfss = 0
          self.sss = 0
      else:
        final_accel = 0
        scaled_accel = 0
        lead_speed = 0.0
        relative_speed = 0.0
        lead_distance = 199
        fusion_distance = 199
        data_source = "no_lead"
        self.sss = 0
        self.rfss = 0

      self.mpc_acc_counter = int(self.mpc_acc_counter + 1) & 0xF

      # 发送控制命令
      can_sends.append(bydcan.acc_cmd(self.packer, self.CP, CS.cam_acc,
                                     getattr(CS, 'mrr_leading_dist', 199),
                                     final_accel, self.rfss, self.sss, CC.longActive, relative_speed,))

      self.apply_accel_last = final_accel
      self.last_acc_frame = self.frame + 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(self.apply_accel_last)
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    self.frame += 1
    return new_actuators, can_sends
