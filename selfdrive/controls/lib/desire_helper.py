from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
from collections import deque

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}


def calculate_lane_width_frog(lane, current_lane, road_edge):
  lane_x, lane_y = np.array(lane.x), np.array(lane.y)
  edge_x, edge_y = np.array(road_edge.x), np.array(road_edge.y)
  current_x, current_y = np.array(current_lane.x), np.array(current_lane.y)

  lane_y_interp = np.interp(current_x, lane_x[lane_x.argsort()], lane_y[lane_x.argsort()])
  road_edge_y_interp = np.interp(current_x, edge_x[edge_x.argsort()], edge_y[edge_x.argsort()])

  distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
  distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge


def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  # 以 t ≈ 1秒前为基准计算到车道/道路边缘的距离
  t = 1.0
  current_lane_y = np.interp(t, ModelConstants.T_IDXS, current_lane.y)
  lane_y = np.interp(t, ModelConstants.T_IDXS, lane.y)
  distance_to_lane = abs(current_lane_y - lane_y)

  road_edge_y = np.interp(t, ModelConstants.T_IDXS, road_edge.y)
  distance_to_road_edge = abs(current_lane_y - road_edge_y)
  distance_to_road_edge_far = abs(current_lane_y - np.interp(2.0, ModelConstants.T_IDXS, road_edge.y))

  lane_valid = lane_prob > 0.5
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, distance_to_road_edge_far, lane_valid


class ExistCounter:
  """
  使用滞后逻辑稳定由于噪声导致的信号波动计数器。
  - 如果 true 持续一定时间以上，则增加 counter（正数）
  - 如果 false 持续一定时间以上，则减少 counter（负数）
  """
  def __init__(self):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(0.2 / DT_MDL)  # 持续约 0.2 秒以上判定为有效

  def update(self, exist_flag: bool):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0
      if self.true_count >= self.threshold:
        self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0
      if self.false_count >= self.threshold:
        self.counter = min(self.counter - 1, -1)
    return self.true_count


from openpilot.selfdrive.carrot.config import UnifiedParams


class DesireHelper:
  def __init__(self):
    self.params = UnifiedParams()
    self.frame = 0

    # 车道变更 / 转向状态
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"  # "none" / "turn" / "lane_change"

    # Desire / 转向相关
    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0   # 转向后一定时间内禁止变道
    self.turn_disable_count = 0     # steeringAngle 非常大时禁止转向

    # 车道 / 道路边缘状态
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_diff = 0.0
    self.lane_width_right_diff = 0.0
    self.distance_to_road_edge_left = 0.0
    self.distance_to_road_edge_right = 0.0
    self.distance_to_road_edge_left_far = 0.0
    self.distance_to_road_edge_right_far = 0.0

    self.lane_exist_left_count = ExistCounter()
    self.lane_exist_right_count = ExistCounter()
    self.lane_width_left_count = ExistCounter()
    self.lane_width_right_count = ExistCounter()
    self.road_edge_left_count = ExistCounter()
    self.road_edge_right_count = ExistCounter()

    self.available_left_lane = False
    self.available_right_lane = False
    self.available_left_edge = False
    self.available_right_edge = False

    self.lane_width_left_queue = deque(maxlen=int(1.0 / DT_MDL))
    self.lane_width_right_queue = deque(maxlen=int(1.0 / DT_MDL))

    self.lane_available_last = False
    self.edge_available_last = False
    self.lane_available_trigger = False
    self.lane_appeared = False
    self.lane_line_info = 0

    # 转向灯 / ATC
    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0  # 0: 无, 1: ATC 运行中, 2: 驾驶员与 ATC 冲突
    self.atc_turn_cnt = 0
    self.atc_turn_cnt_time = 0
    self.reset_atc_turn_cnt_time = 0
    self.atc_blinker_state = BLINKER_NONE
    self.blinker_val = BLINKER_NONE

    # 自动变道相关
    self.auto_lane_change_enable = False
    self.next_lane_change = False
    self.blindspot_detected_counter = 0
    self.lane_change_disable_count = 0.0
    self.lane_change_disable = False
    self.left_sec = -1
    self.dh_left_sec = -1
    self.lane_change_audio_delay = 0.0
    self.lane_change_delay_start = False
    self.driver_lane_change_delay = 0.0
    self.carrot_overtake_cmd = False

    # 保持脉冲
    self.keep_pulse_timer = 0.0

    # 参数
    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneChangeDelay = 0.0
    self.modelTurnSpeedFactor = 0.0
    self.model_turn_speed = 0.0
    self.continuousLaneChange = 0
    self.continuousLaneChangeCnt = 0
    self.continuousLaneChangeInterval = 0
    self.autoEnTurnNewLaneTime = 0
    self.autoEnTurnNewLaneTimeH = 0
    self.newLaneWidthDiff = 0
    self.autoTurnLeft = 0
    self.blinkerMode = 0
    self.autoLaneChangeMinSpeed = 30
    self.disableBlindSpot = False

    # 其他
    self.prev_desire_enabled = False
    self.desireLog = ""
    self.object_detected_count = 0

    # ★ 当前车道概率 (以 Ego 为基准 左/右)
    self.cur_left_prob = 1.0   # laneLineProbs[1]
    self.cur_right_prob = 1.0  # laneLineProbs[2]
    self.current_lane_missing = False
  # ─────────────────────────────────────────────
  #  配置 / 模型相关
  # ─────────────────────────────────────────────

  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
      self.modelTurnSpeedFactor = self.params.get_float("ModelTurnSpeedFactor") * 0.1
      self.continuousLaneChange = self.params.get_int("ContinuousLaneChange")
      self.continuousLaneChangeCnt = self.params.get_int("ContinuousLaneChangeCnt")
      self.continuousLaneChangeInterval = self.params.get_int("ContinuousLaneChangeInterval")
      self.autoEnTurnNewLaneTime = self.params.get_int("AutoEnTurnNewLaneTime")
      self.autoEnTurnNewLaneTimeH = self.params.get_int("AutoEnTurnNewLaneTimeH")
      self.newLaneWidthDiff = self.params.get_int("NewLaneWidthDiff")
      self.autoTurnLeft = self.params.get_int("AutoTurnLeft")
      self.blinkerMode = self.params.get_int("BlinkerMode")
      self.autoLaneChangeMinSpeed = self.params.get_int("AutoLaneChangeMinSpeed")
      self.disableBlindSpot = self.params.get_bool("DisableBlindSpot")

  def _make_model_turn_speed(self, modeldata):
    if self.modelTurnSpeedFactor > 0:
      model_turn_speed = np.interp(self.modelTurnSpeedFactor,
                                   modeldata.velocity.t,
                                   modeldata.velocity.x) * CV.MS_TO_KPH * 1.2
      self.model_turn_speed = self.model_turn_speed * 0.9 + model_turn_speed * 0.1
    else:
      self.model_turn_speed = 200.0

  # ─────────────────────────────────────────────
  #  车道 / 边缘状态计算
  # ─────────────────────────────────────────────

  def _check_lane_state(self, modeldata):
    # 左侧: laneLines[0] vs 当前车道 laneLines[1], roadEdges[0]
    lane_width_left, self.distance_to_road_edge_left, self.distance_to_road_edge_left_far, lane_prob_left = \
      calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0],
                           modeldata.laneLines[1], modeldata.roadEdges[0])

    # 右侧: laneLines[3] vs 当前车道 laneLines[2], roadEdges[1]
    lane_width_right, self.distance_to_road_edge_right, self.distance_to_road_edge_right_far, lane_prob_right = \
      calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3],
                           modeldata.laneLines[2], modeldata.roadEdges[1])

    # 更新车道存在计数器
    self.lane_exist_left_count.update(lane_prob_left)
    self.lane_exist_right_count.update(lane_prob_right)

    # 1秒移动平均 (减少噪声)
    self.lane_width_left_queue.append(lane_width_left)
    self.lane_width_right_queue.append(lane_width_right)

    self.lane_width_left = float(np.mean(self.lane_width_left_queue))
    self.lane_width_right = float(np.mean(self.lane_width_right_queue))

    self.lane_width_left_diff = self.lane_width_left_queue[-1] - self.lane_width_left_queue[0]
    self.lane_width_right_diff = self.lane_width_right_queue[-1] - self.lane_width_right_queue[0]

    # 有效车道/边缘判定
    min_lane_width = 2.5
    self.lane_width_left_count.update(self.lane_width_left > min_lane_width)
    self.lane_width_right_count.update(self.lane_width_right > min_lane_width)
    self.road_edge_left_count.update(self.distance_to_road_edge_left > min_lane_width)
    self.road_edge_right_count.update(self.distance_to_road_edge_right > min_lane_width)

    available_count = int(0.2 / DT_MDL)
    self.available_left_lane = self.lane_width_left_count.counter > available_count
    self.available_right_lane = self.lane_width_right_count.counter > available_count
    self.available_left_edge = self.road_edge_left_count.counter > available_count and self.distance_to_road_edge_left_far > min_lane_width
    self.available_right_edge = self.road_edge_right_count.counter > available_count and self.distance_to_road_edge_right_far > min_lane_width

    self.cur_left_prob = modeldata.laneLineProbs[1]
    self.cur_right_prob = modeldata.laneLineProbs[2]

  # ─────────────────────────────────────────────
  #  模型内 desire 状态 (转向预测等)
  # ─────────────────────────────────────────────

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    # turnLeft + turnRight 概率
    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1

    #if self.turn_desire_state:
    #  self.desire_disable_count = int(2.0 / DT_MDL)
    #else:
    #  self.desire_disable_count = max(0, self.desire_disable_count - 1)

    # 如果 steeringAngle 太大，暂时阻止转向
    if abs(carstate.steeringAngleDeg) > 80:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  #  转向灯 / ATC 状态更新
  # ─────────────────────────────────────────────

  def _update_driver_blinker(self, carstate):
    driver_blinker_state = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    driver_blinker_changed = driver_blinker_state != self.driver_blinker_state
    self.driver_blinker_state = driver_blinker_state

    driver_desire_enabled = driver_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]
    if self.laneChangeNeedTorque < 0:
      # 忽略驾驶员转向灯且不进行变道的设置
      driver_desire_enabled = False

    return driver_blinker_state, driver_blinker_changed, driver_desire_enabled

  def _update_atc_blinker(self, carrotMan, v_ego, driver_blinker_state, driver_blinker_changed):
    """
    基于 ATC 传来的转向/变道命令更新转向灯状态。
    支持连续变道和基于到达时间的变道频率自动规划。
    """
    atc_left_right = False
    fork_left_right = False
    turn_left_right = False
    fork_now = False
    xDistToTurn = carrotMan.xDistToTurn
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # 到转弯点还需要的时间及规划
    left_turn_sec = max(0, carrotMan.xLeftTurnSec)
    turn_need_time = (self.continuousLaneChangeCnt + 1) * self.continuousLaneChangeInterval + 20
    turn_need_time = max(min(left_turn_sec, turn_need_time) - 20, 0)
    lane_change_interval = min(turn_need_time / (self.continuousLaneChangeCnt + 1), self.continuousLaneChangeInterval)
    if lane_change_interval < 2:
      lane_change_interval = 2 if self.continuousLaneChangeInterval >= 2 else self.continuousLaneChangeInterval

    # 1. 处理来自 App 的显式变道命令
    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
      if self.carrot_overtake_cmd:
        atc_left_right = True
      if carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and (carrotMan.carrotCmd == "LANECHANGE" or carrotMan.carrotCmd == "OVERTAKE"):
        self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and (carrotMan.carrotCmd == "LANECHANGE" or carrotMan.carrotCmd == "OVERTAKE"):
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(LANE_CHANGE_TIME_MAX * 2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
      self.atc_turn_cnt = self.continuousLaneChangeCnt
      self.lane_change_disable_count = lane_change_interval
      self.lane_change_disable = False
      if carrotMan.carrotCmd == "OVERTAKE":
        self.carrot_overtake_cmd = True

    # 2. 处理自动转向导航指令 (ATC)
    elif atc_type in ["turn left", "turn right"]:
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if "left" in atc_type else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
        turn_left_right = True
    elif atc_type in ["fork left", "fork right"]:
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if "left" in atc_type else BLINKER_RIGHT
        self.atc_active = 1
        fork_left_right = True
    elif atc_type in ["fork left now", "fork right now"]:
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if "left" in atc_type else BLINKER_RIGHT
        self.atc_active = 1
        fork_now = True
        fork_left_right = True
    elif atc_type in ["atc left", "atc right"]:
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if "left" in atc_type else BLINKER_RIGHT
        self.atc_active = 1
        atc_left_right = True
    else:
      self.atc_active = 0

    # 冲突检测
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2
    elif driver_blinker_state != BLINKER_NONE and driver_blinker_state == atc_blinker_state and driver_blinker_changed:
      if self.atc_active == 2:
        self.atc_active = 0

    atc_desire_enabled = atc_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]

    # 用户干预检测
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      driver_blinker_state = BLINKER_NONE
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 类型切换时的状态重置
    if self.atc_type != atc_type:
      atc_desire_enabled = False
      if fork_now:
        self.atc_turn_cnt = 0
      else:
        self.atc_turn_cnt = self.continuousLaneChangeCnt
      self.lane_change_disable_count = lane_change_interval
      self.lane_change_disable = False
      self.reset_atc_turn_cnt_time = 0

    self.atc_type = atc_type
    self.atc_blinker_state = atc_blinker_state

    return atc_blinker_state, atc_desire_enabled, turn_left_right, lane_change_interval

  # ─────────────────────────────────────────────
  #  转向 / 变道模式分类
  # ─────────────────────────────────────────────

  def _classify_maneuver_type(self, blinker_state, carstate, old_type):
    """
    当转向灯开启时，对当前操作是转向还是变道进行分类。
    - 结合当前速度/减速/车道状态/模型转向状态进行评分。
    """
    if blinker_state == BLINKER_NONE:
      return "none"

    v_kph = carstate.vEgo * CV.MS_TO_KPH
    accel = carstate.aEgo

    # 根据转向灯方向选择要参考的车道/边缘
    if blinker_state == BLINKER_LEFT:
      lane_exist_counter = self.lane_exist_left_count.counter
      lane_available = self.available_left_lane
      edge_available = self.available_left_edge
      lane_prob_side = self.cur_left_prob
      edge_dist = self.distance_to_road_edge_left_far
    else:
      lane_exist_counter = self.lane_exist_right_count.counter
      lane_available = self.available_right_lane
      edge_available = self.available_right_edge
      lane_prob_side = self.cur_right_prob
      edge_dist = self.distance_to_road_edge_right_far

    score_turn = 0

    if v_kph < 30.0:
      score_turn += 1
    elif v_kph < 40.0 and accel < -1.0:
      score_turn += 1

    # 没有车道，道路边缘也比较局促
    if v_kph < 40.0 and not lane_available and not edge_available:
      score_turn += 1

    # 车道不清晰 (路口/三岔路口等)
    if v_kph < 40.0 and lane_exist_counter < int(0.5 / DT_MDL):
      score_turn += 1

    # steeringAngle 较大则认为更接近转向
    #if abs(carstate.steeringAngleDeg) > 45.0:
    #  score_turn += 1

    # 如果模型已经在预测转向，则加权
    if self.turn_desire_state:
      score_turn += 1

    # 如果 ATC 正在引导转向，则加权
    if self.atc_type in ["turn left", "turn right"]:
      score_turn += 2
    elif self.atc_type in ["fork left", "fork right", "atc left", "atc right"]:
      score_turn -= 2  # fork/atc 更倾向于变道

    # ★ 如果道路边缘足够远 (判定为路口/开阔空间)，给转向加分
    edge_far = edge_dist > 4.0  # 调优参数 (4~6m 比较合适)
    #if edge_far:
    #  score_turn += 1
      
    current_lane_missing = lane_prob_side < 0.3
    self.current_lane_missing = current_lane_missing
    # 调优参数：转向评分阈值
    if score_turn >= 2:
      #if current_lane_missing and edge_far:
      if edge_far:
        return "turn"
      else:
        return old_type
    else:
      return "lane_change"

  # ─────────────────────────────────────────────
  #  主更新例程
  # ─────────────────────────────────────────────

  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.frame += 1
    self._update_params_periodic()
    self._make_model_turn_speed(modeldata)

    # 计数器递减
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.driver_lane_change_delay = max(0, self.driver_lane_change_delay - DT_MDL)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)
    self.lane_change_audio_delay = max(0, self.lane_change_audio_delay - DT_MDL)
    if self.lane_change_delay_start:
      left_sec = min(10, int(self.lane_change_delay))
      if self.left_sec != left_sec and self.lane_change_audio_delay <= 0:
        self.dh_left_sec = left_sec
        self.left_sec = left_sec

    if self.lane_change_disable:
      self.lane_change_disable_count = max(0, self.lane_change_disable_count - DT_MDL)
      if self.lane_change_disable_count <= 0:
        self.lane_change_disable = False
      left_sec = min(10, int(self.lane_change_disable_count))
      if self.left_sec != left_sec and self.lane_change_audio_delay <= 0:
        self.dh_left_sec = left_sec
        self.left_sec = left_sec

    self.blindspot_detected_counter = max(0, self.blindspot_detected_counter - 1)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # 车道 / desire 状态更新
    self._check_lane_state(modeldata)
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # 驾驶员转向灯
    driver_blinker_state, driver_blinker_changed, driver_desire_enabled = self._update_driver_blinker(carstate)

    # BSD 设置
    ignore_bsd = (self.laneChangeBsd < 0)
    block_lanechange_bsd = (self.laneChangeBsd == 1)

    # ATC 转向灯
    atc_blinker_state, atc_desire_enabled, turn_left_right, lane_change_interval = self._update_atc_blinker(carrotMan, v_ego, driver_blinker_state, driver_blinker_changed)

    # 最终转向灯 / Desire enabled 判定
    desire_enabled = driver_desire_enabled or atc_desire_enabled
    blinker_state = driver_blinker_state if driver_desire_enabled else atc_blinker_state

    # lane_line_info (用于 HUD 等)
    lane_line_info = carstate.leftLaneLine if blinker_state == BLINKER_LEFT else carstate.rightLaneLine

    # BSD / 周边车辆感应
    if desire_enabled:
      lane_exist_counter = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
      lane_available = self.available_left_lane if blinker_state == BLINKER_LEFT else self.available_right_lane
      edge_available = self.available_left_edge if blinker_state == BLINKER_LEFT else self.available_right_edge
      self.lane_appeared = self.lane_appeared or lane_exist_counter == int(0.2 / DT_MDL)

      radar = radarState.leadLeft if blinker_state == BLINKER_LEFT else radarState.leadRight
      side_object_dist = radar.dRel + radar.vLead * 4.0 if radar.status else 255
      object_detected = side_object_dist < v_ego * 3.0
      if object_detected:
        self.object_detected_count = max(1, self.object_detected_count + 1)
      else:
        self.object_detected_count = min(-1, self.object_detected_count - 1)

      lane_line_info_edge_detect = (lane_line_info % 10 in [0, 5] and self.lane_line_info not in [0, 5])
      self.lane_line_info = lane_line_info % 10
    else:
      lane_exist_counter = 0
      lane_available = True
      edge_available = True
      self.lane_appeared = False
      self.lane_available_trigger = False
      self.object_detected_count = 0
      lane_line_info_edge_detect = False
      self.lane_line_info = lane_line_info % 10

    # 基于车道/边缘的变道可能判定
    lane_change_available = (lane_available or edge_available) and lane_line_info < 20  # 小于 20 为白色线

    # 根据 lane_available 变化 & 宽度变化计算 lane_available_trigger
    self.lane_available_trigger = False
    if blinker_state == BLINKER_LEFT:
      lane_width_diff = self.lane_width_left_diff
      distance_to_road_edge = self.distance_to_road_edge_left
      lane_width_side = self.lane_width_left
    else:
      lane_width_diff = self.lane_width_right_diff
      distance_to_road_edge = self.distance_to_road_edge_right
      lane_width_side = self.lane_width_right

    if lane_width_diff > 0.8 and (lane_width_side < distance_to_road_edge):
      self.lane_available_trigger = True

    edge_availabled = (not self.edge_available_last and edge_available)
    side_object_detected = self.object_detected_count > -0.3 / DT_MDL
    self.lane_appeared = self.lane_appeared and distance_to_road_edge < 4.0

    # 自动变道触发
    if self.carrot_lane_change_count > 0:
      auto_lane_change_blocked = False
      auto_lane_change_trigger = lane_change_available
    else:
      auto_lane_change_blocked = ((atc_blinker_state == BLINKER_LEFT) and (driver_blinker_state != BLINKER_LEFT))
      self.auto_lane_change_enable = self.auto_lane_change_enable and not auto_lane_change_blocked
      auto_lane_change_trigger = self.auto_lane_change_enable and edge_available and (self.lane_available_trigger or self.lane_appeared) and not side_object_detected
      self.desireLog = f"L:{self.auto_lane_change_enable},{auto_lane_change_blocked},E:{lane_available},{edge_available},A:{self.lane_available_trigger},{self.lane_appeared},{lane_width_diff:.1f},{lane_width_side:.1f},{distance_to_road_edge:.1f}={auto_lane_change_trigger}"

    # 主状态机

    # 0) lateral 断开或持续时间过长则重置
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    # 1) 转向后一定时间内不执行任何操作
    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    else:
      # 转向灯开启时，先分类当前操作是转向还是变道
      if desire_enabled:
        new_type = self._classify_maneuver_type(blinker_state, carstate, self.maneuver_type)
      else:
        new_type = "none"

      # ★ 1) 原本是变道，但新判定符合转向条件 + 车道缺失 → 允许强制切换
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in [LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting]:
        # 变道过程中如果满足条件也可切换到转向
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off  # 重置 FSM 后进入转向路径
      # ★ 2) 其他情况仅在 off/pre 状态下允许变更模式
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ 转向 (TURN) 模式处理 ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires: # and not carstate.standstill:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          # 为了兼容性，将 lane_change_direction 也设置为与转向一致
          self.lane_change_direction = self.turn_direction
          self.blinker_val = blinker_state if turn_left_right else BLINKER_NONE

      # ─ 变道 (Lane Change) FSM 处理 ─
      else:
        self.turn_direction = TurnDirection.none

        # LaneChangeState.off
        if self.lane_change_state == LaneChangeState.off:
          if desire_enabled and not self.prev_desire_enabled and not below_lane_change_speed:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.lane_change_delay = self.laneChangeDelay

            # 如果不是最外侧车道，则禁用 ATC 自动变道
            lane_exist_counter_side = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
            self.auto_lane_change_enable = False if lane_exist_counter_side > 0 or lane_change_available else True
            self.next_lane_change = False
            self.lane_change_delay_start = False
            self.lane_change_disable = False

        # LaneChangeState.preLaneChange
        elif self.lane_change_state == LaneChangeState.preLaneChange:
          self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right
          dir_map = {
            LaneChangeDirection.left: (carstate.steeringTorque > 0, carstate.leftBlindspot),
            LaneChangeDirection.right: (carstate.steeringTorque < 0, carstate.rightBlindspot),
          }
          torque_cond, blindspot_cond = dir_map.get(self.lane_change_direction, (False, False))
          torque_applied = carstate.steeringPressed and torque_cond
          blindspot_detected = blindspot_cond

          # 如果车道持续一定时间不可见，允许自动变道
          lane_exist_counter_side = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
          if not lane_available or lane_exist_counter_side < int(2.0 / DT_MDL):
            self.auto_lane_change_enable = True

          # BSD 盲区检测
          if blindspot_detected and not ignore_bsd:
            self.blindspot_detected_counter = int(1.5 / DT_MDL)

          if not desire_enabled or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            # 变道开始条件
            if ((lane_change_available and self.lane_change_delay == 0) or lane_line_info_edge_detect) and not self.lane_change_disable:
              if self.blindspot_detected_counter > 0 and not ignore_bsd:
                if torque_applied and not block_lanechange_bsd:
                  self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.laneChangeNeedTorque > 0 or self.next_lane_change:
                if torque_applied:
                  self.lane_change_state = LaneChangeState.laneChangeStarting
              elif driver_desire_enabled:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif torque_applied or auto_lane_change_trigger or lane_line_info_edge_detect:
                self.lane_change_state = LaneChangeState.laneChangeStarting

        # LaneChangeState.laneChangeStarting
        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          # 原车道线在 1.0 秒内逐渐淡出 (从 0.5 秒拉长)
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 1 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing

        # LaneChangeState.laneChangeFinishing
        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          # 车道线在 1.0 秒内逐渐恢复
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              self.next_lane_change = True
              # 连续变道处理
              if self.atc_turn_cnt > 0:
                self.atc_turn_cnt -= 1
                self.lane_change_disable_count = lane_change_interval
                self.lane_change_disable = True
                self.lane_change_audio_delay = 2.0
            else:
              self.lane_change_state = LaneChangeState.off
              self.atc_turn_cnt = 0

    # lane_change_timer 管理
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.lane_available_last = lane_available
    self.edge_available_last = edge_available

    self.prev_desire_enabled = desire_enabled

    # 如果驾驶员向相反方向猛打方向盘，则取消该变道/转向
    steering_pressed_cancel = carstate.steeringPressed and \
                               ((carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
                                (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT))
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    # 决定最终 desire
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # 保持脉冲 (在 LaneChangeState.preLaneChange 状态下维持)
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
