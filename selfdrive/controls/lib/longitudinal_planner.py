#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_speed_error, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params


LON_MPC_STEP = 0.2  # 第一步为 0.2s
A_CRUISE_MIN = -2.0 #-1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5

# 弯道查找表
_A_TOTAL_MAX_V = [2.4, 4.8] #[1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]
LAT_WEIGHT = 0.7


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # 根据数据拟合自 xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns_org(v_ego, angle_steers, a_target, CP):
  """
  此函数返回允许的限制纵向加速度，取决于现有的横向加速度
  这应该避免在弯道丢失目标时加速
  """
  # FIXME: 此横向加速度计算函数不正确，应使用 VehicleModel
  # 如果我们这样做，弯道查找表也应更新
  steer_abs = abs(angle_steers)
  if v_ego > 20 or (v_ego > 25 and steer_abs < 3.0):
    return a_target
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase) * LAT_WEIGHT
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]

def limit_accel_in_turns(v_ego, curvature, a_target, a_lat_max):
  """
  v_ego    : m/s
  curvature: 1/m  (来自转向的曲率，包含符号)
  a_target : [a_min, a_max]
  a_lat_max: 允许的最大横向加速度 (例如: 6.0 ~ 8.0 m/s^2)

  return   : [a_min, 受限的 a_max]
  """
  # 极低速时无需限制
  if v_ego < 0.1 or a_lat_max <= 0.0:
    return a_target

  # 横向加速度
  a_y = v_ego * v_ego * curvature        # m/s^2
  a_y_abs = abs(a_y)
  a_lat_max_abs = abs(a_lat_max)

  # 假设 a_total^2 = a_x^2 + a_y^2 <= a_lat_max^2
  # 计算剩余的 a_x 裕量
  if a_y_abs >= a_lat_max_abs:
    a_x_allowed = 0.0
  else:
    a_x_allowed = math.sqrt(a_lat_max_abs**2 - a_y_abs**2)

  # 仅减小 a_target = [min, max] 中的 max 值
  return [a_target[0], min(a_target[1], a_x_allowed)]

class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_v_target_now = 0.0
    self.output_j_target_now = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.vCluRatio = 1.0

    self.v_cruise_kph = 0.0

    self.params = Params()

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, carrot):
    self.mpc.mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    self.mpc.mode = carrot.mode
    v_cruise = carrot.v_cruise

    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio

    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # 未激活或用户控制速度时重置当前状态
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM 巡航速度可能会延迟几个周期更新，检查是否已初始化
    reset_state = reset_state or not v_cruise_initialized or carrot.soft_hold_active

    # 用户控制速度或静止时不设变化代价
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits = [A_CRUISE_MIN, carrot.get_carrot_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      #accel_limits_turns = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_limits, self.CP)
      a_lat_max = 3.8 # 从 3.2 提升至 3.8，减少非必要的弯道重刹，体感更丝滑
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['controlsState'].desiredCurvature, accel_limits, a_lat_max)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # 将 aEgo 限制在巡航范围内，以防止激活时产生剧烈加速
      self.a_desired = np.clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## carrot
      accel_limits_turns[0] = 0.0 ## carrot
      accel_limits_turns[1] = 0.0

    # 防止发散，平滑当前 v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # 在低速时不修剪，因为 throttle_prob 不考虑蠕行
    if self.params.get_int("CommaLongAcc") > 0:
      self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    else:
      self.allow_throttle = True

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_limits_turns[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_limits_turns[1], clipped_accel_coast])
      accel_limits_turns[1] = min(accel_limits_turns[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0
    # 限制修剪，不能在范围外初始化 MPC
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality, jerk_factor = carrot.jerk_factor_apply, a_change_cost_starting = carrot.aChangeCostStarting)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(carrot, reset_state, sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter 仅因雷达不稳定而需要，一旦雷达被移除即可移除
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # 插值 0.05 秒并保存为下一次迭代的起点
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    longitudinalActuatorDelay = self.params.get_float("LongActuatorDelay")*0.01
    vEgoStopping = self.params.get_float("VEgoStopping") * 0.01
    action_t =  longitudinalActuatorDelay + DT_MDL

    output_a_target_mpc, output_should_stop_mpc, output_v_target_mpc, _ = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                         action_t=action_t, vEgoStopping=vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_v_target_now_e2e = sm['modelV2'].action.desiredVelocity

    if self.mpc.mode == 'acc':
      output_a_target = output_a_target_mpc
      output_v_target_now = output_v_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      output_v_target_now = min(output_v_target_mpc, output_v_target_now_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    self.output_a_target = output_a_target
    self.output_v_target_now = output_v_target_now
    self.output_j_target_now = self.j_desired_trajectory[0]

  def publish(self, sm, pm, carrot):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.vTargetNow = float(self.output_v_target_now)
    longitudinalPlan.jTargetNow = float(self.output_j_target_now)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    longitudinalPlan.xState = carrot.xState.value
    longitudinalPlan.trafficState = carrot.trafficState.value
    longitudinalPlan.cruiseTarget = self.v_cruise_kph
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.desiredDistance = float(self.mpc.desired_distance)
    longitudinalPlan.events = carrot.events.to_msg()
    longitudinalPlan.myDrivingMode = carrot.myDrivingMode.value

    pm.send('longitudinalPlan', plan_send)
