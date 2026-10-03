import math
import numpy as np

from cereal import car
from openpilot.common.conversions import Conversions as CV

from opendbc.car import structs
GearShifter = structs.CarState.GearShifter


# 警告：此值是根据模型的训练分布确定的，
#       超过此速度的模型预测可能不可预测
# V_CRUISE 单位为 kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # 在此处舍入以避免递增设定速度时的舍入误差


ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # 如果 stock cruise 被完全禁用，我们可以使用我们自己的设定速度逻辑
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # 处理按钮按下。TODO: 应该在 state_control 中处理，但 decelCruise 按下
    # 会同时产生启用和更改速度的效果，在状态转换后进行检查
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # 结束长按
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # 退出静止状态按下 resume 时不调整速度
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # 如果自按钮松开以来我们已经启用，则不调整速度（某些端口在上升沿启用）
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # 部分间隔
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # 如果在踩油门时按下 set，将巡航速度限制为 vEgo 的最小值
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # 为仍在按下的按钮递增计时器
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # 在按钮按下状态改变时启动/结束计时器并存储当前状态
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # 初始化由 PCM 处理
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph



from openpilot.common.params import Params

class VCruiseCarrot:
  def __init__(self, CP):
    self.CP = CP
    self.frame = 0
    self.params_memory = Params("/dev/shm/params")
    self.params = Params()
    self.v_cruise_kph = 20 #V_CRUISE_UNSET
    self.v_cruise_cluster_kph = 20 #V_CRUISE_UNSET
    self.v_cruise_kph_last = 20

    self.enabled_last = False

    self.long_pressed = False
    self.button_cnt = 0
    self.button_prev = ButtonType.unknown
    self.button_long_time = 40

    self.is_metric = True

    self.v_ego_kph_set = 0
    self._cruise_speed_min, self._cruise_speed_max = 5, 161
    self._cruise_speed_unit = 10
    self._cruise_speed_unit_basic = 1
    self._cruise_button_mode = 2
    self._cancel_button_mode = 0
    self._lfa_button_mode = 0

    self._gas_pressed_count = 0
    self._gas_pressed_count_last = 0
    self._gas_pressed_value = 0
    self._gas_tok_timer = int(0.4 / 0.01) # 0.4 sec
    self._gas_tok = False
    self._brake_pressed_count = 0
    self._soft_hold_count = 0
    self._soft_hold_active = 0
    self._cruise_ready = False
    self._cruise_cancel_state = False
    self._pause_auto_speed_up = False
    self._activate_cruise = 0
    self._lat_enabled = self.params.get_int("AutoEngage") > 0
    self._v_cruise_kph_at_brake = 0
    self.cruise_state_available_last = False

    self._paddle_decel_active = False
    self.carrot_cruise_active = False
    self._v_cruise_kph_pre_kpfe = 0
    self._v_cruise_manual_max = 100.0

    #self.events = []
    self.xState = 0
    self.trafficState = 0
    self.aTarget = 0
    self.nRoadLimitSpeed = 30
    self.desiredSpeed = 250
    self.road_limit_kph = 30
    self.nRoadLimitSpeed_last = 30

    self.carrot_cmd_index_last = 0
    self.carrot_cmd_index = 0
    self.carrot_cmd = ""
    self.carrot_arg = ""


    self._cancel_timer = 0
    self.d_rel = 0
    self.v_rel = 0
    self.v_lead_kph = 0

    self._log_timer = 0
    self._log_timeout = int(3/0.01)
    self.log = ""

    self.autoCruiseControl = 0
    self.autoCruiseControl_cancel_timer = 0
    self.AutoSpeedUptoRoadSpeedLimit = 0.0

    self.useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed")
    self.useLaneLineSpeedApply = self.useLaneLineSpeed


  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
        #self.event = -1
    else:
      self.log = log
      #self.event = event
      self._log_timer = self._log_timeout

  def update_params(self, is_metric):
    unit_factor = 1.0 if is_metric else CV.MPH_TO_KPH
    if self.frame % 10 == 0:
      self.autoCruiseControl = self.params.get_int("AutoCruiseControl") * unit_factor
      self.autoGasTokSpeed = self.params.get_int("AutoGasTokSpeed") * unit_factor
      self.autoGasSyncSpeed = self.params.get_bool("AutoGasSyncSpeed") * unit_factor
      self.autoSpeedUptoRoadSpeedLimit = self.params.get_float("AutoSpeedUptoRoadSpeedLimit") * 0.01
      self.autoRoadSpeedAdjust = self.params.get_float("AutoRoadSpeedAdjust") * 0.01
      self.smartSpeedControl = self.params.get_int("CarrotSmartSpeedControl")

      useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed") * unit_factor
      if self.useLaneLineSpeed != useLaneLineSpeed:
        self.useLaneLineSpeedApply = useLaneLineSpeed
      self.useLaneLineSpeed = useLaneLineSpeed

      self.speed_from_pcm = self.params.get_int("SpeedFromPCM")
      self._cruise_speed_unit = self.params.get_int("CruiseSpeedUnit")
      self._cruise_speed_unit_basic = self.params.get_int("CruiseSpeedUnitBasic")
      self._paddle_mode = self.params.get_int("PaddleMode")
      self._cruise_button_mode = self.params.get_int("CruiseButtonMode")
      self._cancel_button_mode = self.params.get_int("CancelButtonMode")
      self._lfa_button_mode = self.params.get_int("LfaButtonMode")
      self.autoRoadSpeedLimitOffset = self.params.get_int("AutoRoadSpeedLimitOffset")
      self.autoNaviSpeedSafetyFactor = self.params.get_float("AutoNaviSpeedSafetyFactor") * 0.01
      self.cruiseOnDist = self.params.get_float("CruiseOnDist") * 0.01
      cruiseSpeed1 = self.params.get_float("CruiseSpeed1") * unit_factor
      cruiseSpeed2 = self.params.get_float("CruiseSpeed2") * unit_factor
      cruiseSpeed3 = self.params.get_float("CruiseSpeed3") * unit_factor
      cruiseSpeed4 = self.params.get_float("CruiseSpeed4") * unit_factor
      cruiseSpeed5 = self.params.get_float("CruiseSpeed5") * unit_factor
      if cruiseSpeed1 <= 0:
        if self.autoRoadSpeedLimitOffset < 0:
          cruiseSpeed1 = self.nRoadLimitSpeed * self.autoNaviSpeedSafetyFactor
        else:
          cruiseSpeed1 = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset
      self._cruise_speed_table = [cruiseSpeed1, cruiseSpeed2, cruiseSpeed3, cruiseSpeed4, cruiseSpeed5]

  def update_v_cruise(self, CS, sm, is_metric):
    self._add_log("")
    self.update_params(is_metric)
    self.frame += 1
    if CS.gearShifter != GearShifter.drive:
      self.autoCruiseControl_cancel_timer = 20 * 100  # 20 sec
    else:
      self.autoCruiseControl_cancel_timer = max(0, self.autoCruiseControl_cancel_timer - 1)

    CC = sm['carControl']
    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      self.nRoadLimitSpeed = carrot_man.nRoadLimitSpeed
      self.desiredSpeed = carrot_man.desiredSpeed
      self.carrot_cmd_index = carrot_man.carrotCmdIndex
      self.carrot_cmd = carrot_man.carrotCmd
      self.carrot_arg = carrot_man.carrotArg
    if sm.alive['longitudinalPlan']:
      lp = sm['longitudinalPlan']
      self.xState = lp.xState
      self.trafficState = lp.trafficState
      self.aTarget = lp.aTarget
    if sm.alive['radarState']:
      lead = sm['radarState'].leadOne
      self.d_rel = lead.dRel if lead.status else 0
      self.v_rel = lead.vRel if lead.status else 0
      self.v_lead_kph = lead.vLeadK * CV.MS_TO_KPH if lead.status else 0
    if sm.alive['modelV2']:
      self.model_v_kph = sm['modelV2'].velocity.x[0] * CV.MS_TO_KPH
    else:
      self.model_v_kph = 0

    self.v_cruise_kph_last = self.v_cruise_kph
    self.is_metric = is_metric

    self._cancel_timer = max(0, self._cancel_timer - 1)

    #self.events = []
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    self._activate_cruise = 0
    self._prepare_brake_gas(CS, CC)
    if CC.enabled:
      self._cruise_ready = False
    v_cruise_kph = self._update_cruise_buttons(CS, CC, self.v_cruise_kph)

    if self._activate_cruise > 0:
      #self.events.append(EventName.buttonEnable)
      self._cruise_ready = False
    elif self._activate_cruise < 0:
      #self.events.append(EventName.buttonCancel)
      self._cruise_ready = True if self._activate_cruise == -2 else False

    if CS.cruiseState.available:
      if not self.cruise_state_available_last:
        self._lat_enabled = True
      if not self.CP.pcmCruise:
        # 如果 stock cruise 被完全禁用，我们可以使用我们自己的设定速度逻辑
        self.v_cruise_kph = np.clip(v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        if self.speed_from_pcm == 1:
          self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        else:
          self.v_cruise_kph = np.clip(v_cruise_kph, 30, self._cruise_speed_max)
          self.v_cruise_cluster_kph = self.v_cruise_kph
    else:
      self.v_cruise_kph = np.clip(v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max) 
      self.v_cruise_cluster_kph = self.v_cruise_kph 
      if self.cruise_state_available_last: # 如果 cruiseState.available 至少曾为 True 一次
       self._lat_enabled = False

    self.cruise_state_available_last = CS.cruiseState.available
    self.enabled_last = CC.enabled

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    return
    # 初始化由 PCM 处理
    if self.CP.pcmCruise and self.speed_from_pcm == 1:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else CS.vEgoCluster * CV.MS_TO_KPH

    v_ego_kph = int(round(np.clip(CS.vEgoCluster * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))
    print(CS.buttonEvents)
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents): # and self.v_cruise_initialized:
      self.v_cruise_kph = max(self._v_cruise_kph_at_brake, v_ego_kph) if self._v_cruise_kph_at_brake > 0 else self.v_cruise_kph_last
      self._add_log(f"{self.v_cruise_kph},{self._v_cruise_kph_at_brake} Cruise resume")
    else:
      self.v_cruise_kph = v_ego_kph
      self._add_log(f"{self.v_cruise_kph} Cruise Set")

    self.v_cruise_kph = np.clip(self.v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
    self.v_cruise_cluster_kph = self.v_cruise_kph
    self._v_cruise_manual_max = self.v_cruise_kph

  def _prepare_buttons(self, CS, v_cruise_kph):
    button_kph = v_cruise_kph
    button_type = 0
    buttonEvents = CS.buttonEvents

    # 单次点击固定 ±1（公制 1km/h，英制 1mph）；长按每步 ±10
    SPEED_UP_UNIT = 1.0
    SPEED_DOWN_UNIT = 1.0
    V_CRUISE_DELTA = 10
    is_metric = self.is_metric

    # 长按跟踪
    if self.button_cnt > 0:
      self.button_cnt += 1

    # 安全超时防抖逻辑：防止任何特殊情况下按钮释放事件丢失，最大计数限制为 300 帧 (3秒)
    if self.button_cnt > 300:
      self.button_cnt = 0
      self.long_pressed = False

    for b in buttonEvents:
      bt = b.type

      if bt in [ButtonType.paddleLeft, ButtonType.paddleRight] and b.pressed:
        # 拨片：立即发生事件
        button_type = bt
        self.long_pressed = False
        self.button_cnt = 0
        continue

      if b.pressed and self.button_cnt == 0 and bt in [
        ButtonType.accelCruise, ButtonType.decelCruise,
        ButtonType.gapAdjustCruise, ButtonType.cancel,
        ButtonType.lfaButton
      ]:
        self.button_cnt = 1
        self.button_prev = bt
        self.button_long_time = 40 if bt in [ButtonType.accelCruise, ButtonType.decelCruise] else 70

      elif not b.pressed and self.button_cnt > 0:
        if self.button_prev == ButtonType.cancel:
          button_type = self.button_prev
        elif not self.long_pressed:
          if self.button_prev == ButtonType.accelCruise:
            unit = SPEED_UP_UNIT if is_metric else SPEED_UP_UNIT * CV.MPH_TO_KPH
            button_kph += unit
          elif self.button_prev == ButtonType.decelCruise:
            unit = SPEED_DOWN_UNIT if is_metric else SPEED_DOWN_UNIT * CV.MPH_TO_KPH
            button_kph -= unit
          button_type = self.button_prev
        self.long_pressed = False
        self.button_cnt = 0

    # 长按处理
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      bt = self.button_prev

      if bt in [ButtonType.accelCruise, ButtonType.decelCruise]:
        # 长按：固定 ±10（公制 km/h / 英制 mph 换算），不再吸附到 10 的整数倍
        step = V_CRUISE_DELTA if is_metric else V_CRUISE_DELTA * CV.MPH_TO_KPH
        button_kph += step if bt == ButtonType.accelCruise else -step
        button_type = bt
        self.button_cnt %= self.button_long_time
      else: 
        if self.button_cnt < self.button_long_time + 2:
          button_type = bt

    return button_kph, button_type, self.long_pressed

  def _carrot_command(self, v_cruise_kph, button_type, long_pressed):
    carrot_speed = self.params_memory.get_int("CarrotSpeed")
    if carrot_speed != 0:
      if carrot_speed > 0:
        if self.smartSpeedControl in [1,3]:
          v_cruise_kph = max(carrot_speed, v_cruise_kph)
      else:
        if self.smartSpeedControl == 3:
          v_cruise_kph = -carrot_speed
        elif self.smartSpeedControl == 2:
          v_cruise_kph = min(-carrot_speed, v_cruise_kph)
      self.params_memory.put_int_nonblocking("CarrotSpeed", 0)
      self._add_log(f"Carrot speed set to {v_cruise_kph}")
    if self.carrot_cmd_index_last != self.carrot_cmd_index:
      self.carrot_cmd_index_last = self.carrot_cmd_index
      print(f"Carrot command(cruise.py): {self.carrot_cmd} {self.carrot_arg}")
      if self.carrot_cmd == "CRUISE":
        if self.carrot_arg == "OFF":
          self._cruise_control(-2, -1, "Cruise off (carrot command)")
        elif self.carrot_arg == "ON":
          self._cruise_control(1, -1, "Cruise on (carrot command)")
        elif self.carrot_arg == "GO":
          if button_type == 0:
            button_type = ButtonType.accelCruise
            long_pressed = False
            self._add_log("Cruise accelCruise (carrot command)")
        elif self.carrot_arg == "STOP":
          v_cruise_kph = 5
          self._add_log("Cruise stop (carrot command)")

      elif self.carrot_cmd == "SPEED":
        if self.carrot_arg == "UP":
          v_cruise_kph = self._auto_speed_up(v_cruise_kph)
          self._add_log("Cruise speed up (carrot command)")
        elif self.carrot_arg == "DOWN":
          if v_cruise_kph > 20:
            v_cruise_kph -= 10
            self._add_log("Cruise speed downup (carrot command)")
        else:
          speed_kph = int(self.carrot_arg)
          if 0 < speed_kph < 200:
            v_cruise_kph = speed_kph
            self._add_log(f"Cruise speed set to {v_cruise_kph} (carrot command)")

    return v_cruise_kph, button_type, long_pressed

  def _update_cruise_buttons(self, CS, CC, v_cruise_kph):
    button_kph, button_type, long_pressed = self._prepare_buttons(CS, v_cruise_kph)

    v_cruise_kph, button_type, long_pressed = self._carrot_command(v_cruise_kph, button_type, long_pressed)

    if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
      self._paddle_decel_active = False
      self._v_cruise_kph_pre_kpfe = 0
      self._v_cruise_manual_max = button_kph
      if self.autoCruiseControl_cancel_timer > 0:
        self._add_log(f"AutoCruiseControl cancel timer RESET {button_type}")
        self.autoCruiseControl_cancel_timer = 0
      if self._cruise_cancel_state:
        self._add_log(f"Cruise Cancel state RESET {button_type}")
        self._cruise_cancel_state = False

    if not long_pressed:
      if button_type == ButtonType.accelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = False
        if self._soft_hold_active > 0:
          self._soft_hold_active = 0
        elif self._cruise_ready or not CC.enabled or CS.cruiseState.standstill or self.carrot_cruise_active:
          if False: #self._cruise_button_mode in [2, 3]:
            road_limit_kph = self.nRoadLimitSpeed * self.autoSpeedUptoRoadSpeedLimit
            if road_limit_kph > 1.0:
              v_cruise_kph = max(v_cruise_kph, road_limit_kph)
        elif self._v_cruise_kph_at_brake > 0 and v_cruise_kph < self._v_cruise_kph_at_brake:
          v_cruise_kph = self._v_cruise_kph_at_brake
          self._v_cruise_kph_at_brake = 0
        elif self._cruise_button_mode == 0:
          v_cruise_kph = button_kph
        else:
          # 单次点击固定 +1（不再吸附到 10 的整数倍）
          v_cruise_kph = button_kph
        self.carrot_cruise_active = False

      elif button_type == ButtonType.decelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = True

        if self._soft_hold_active > 0:
          self._cruise_control(-1, -1, "Cruise off,softhold mode (decelCruise)")
        elif self._cruise_ready:
          self._paddle_decel_active = True
          pass
        elif not CC.enabled:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self.v_ego_kph_set > v_cruise_kph + 2 and self._cruise_button_mode in [2, 3]:
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
        elif self._cruise_button_mode in [0, 1, 2, 3]:
          v_cruise_kph = button_kph
        elif self.v_ego_kph_set < 1.0:
          self.carrot_cruise_active = True
        elif self.v_ego_kph_set > self._cruise_speed_min and v_cruise_kph > self.v_ego_kph_set:
          v_cruise_kph = self.v_ego_kph_set
        else:
          self.carrot_cruise_active = True
        self._v_cruise_kph_at_brake = 0

      elif button_type == ButtonType.gapAdjustCruise:
        longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
        if CS.pcmCruiseGap == 0:
          personality = (self.params.get_int('LongitudinalPersonality') - 1) % longitudinalPersonalityMax
        else:
          personality = np.clip(CS.pcmCruiseGap - 1, 0, longitudinalPersonalityMax)
        self.params.put_int_nonblocking('LongitudinalPersonality', personality)
      elif button_type == ButtonType.lfaButton:
        if self._lfa_button_mode == 0:
          self._lat_enabled = not self._lat_enabled
          self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")
        elif self._lfa_button_mode == 2:
          self.carrot_cruise_active = True
        else:
          if False: # 需要修改...
            self._paddle_decel_active = False
          else:
            self._paddle_decel_active = True
        print("lfaButton")
      elif button_type == ButtonType.cancel:
        self._paddle_decel_active = False
        if self._cancel_button_mode in [1]:
          self._lat_enabled = False
          self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")
        self._cruise_cancel_state = True
    else:
      if button_type == ButtonType.accelCruise:
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.decelCruise:
        self._pause_auto_speed_up = True
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.gapAdjustCruise:
        self.params.put_int_nonblocking("MyDrivingMode", self.params.get_int("MyDrivingMode") % 4 + 1) # 1,2,3,4 (1:eco, 2:safe, 3:normal, 4:high speed)
      elif button_type == ButtonType.lfaButton:
        useLaneLineSpeed = max(1, self.useLaneLineSpeed)
        self.useLaneLineSpeedApply = useLaneLineSpeed if self.useLaneLineSpeedApply == 0 else 0

      elif button_type == ButtonType.cancel:
        self._cruise_cancel_state = True
        self._lat_enabled = False
        self._paddle_decel_active = False
        self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")

    if self._paddle_mode > 0 and button_type in [ButtonType.paddleLeft, ButtonType.paddleRight]:  # paddle button
      self._cruise_control(-2, -1, "Cruise off & Ready (paddle)")
      if self._paddle_mode == 2:
        self._paddle_decel_active = True
    elif self._paddle_decel_active:
      if not CC.enabled:
        self._cruise_control(1, -1, "Cruise on (paddle decel)")

    v_cruise_kph = self._update_cruise_state(CS, CC, v_cruise_kph)
    return v_cruise_kph

  def _v_cruise_desired(self, CS, v_cruise_kph):
    if self._cruise_button_mode == 3:
      for speed in self._cruise_speed_table:
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
      else:
        v_cruise_kph = ((v_cruise_kph // self._cruise_speed_unit) + 1) * self._cruise_speed_unit

    elif v_cruise_kph < 30: 
      v_cruise_kph = 30 
    else:
      for speed in range (40, 160, self._cruise_speed_unit):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return v_cruise_kph

  def _auto_speed_up(self, v_cruise_kph):
    road_limit_kph = self.nRoadLimitSpeed * self.autoSpeedUptoRoadSpeedLimit
    if road_limit_kph < 1.0:
      return v_cruise_kph

    if self.autoRoadSpeedLimitOffset > 0:
      self._v_cruise_kph_at_brake = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset

    if not self._pause_auto_speed_up and self.v_lead_kph + 5 > v_cruise_kph and v_cruise_kph < road_limit_kph and self.d_rel < 60:
      v_cruise_kph = min(v_cruise_kph + 5, road_limit_kph)
    elif self.autoRoadSpeedAdjust < 0 and self.nRoadLimitSpeed != self.nRoadLimitSpeed_last:  # 如果道路限速改变，则将速度调整为新的限速
      if self.autoRoadSpeedLimitOffset < 0:
        v_cruise_kph = self.nRoadLimitSpeed * self.autoNaviSpeedSafetyFactor
      else:
        v_cruise_kph = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset
    elif self.nRoadLimitSpeed < self.nRoadLimitSpeed_last and self.autoRoadSpeedAdjust > 0:
      new_road_limit_kph = self.nRoadLimitSpeed * self.autoRoadSpeedAdjust + v_cruise_kph * (1 - self.autoRoadSpeedAdjust)
      self._add_log(f"AutoSpeed change {v_cruise_kph} -> {new_road_limit_kph:.1f}")
      v_cruise_kph = min(v_cruise_kph, new_road_limit_kph)
    self.road_limit_kph = road_limit_kph
    self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
    return v_cruise_kph

  def _cruise_control(self, enable, cancel_timer, reason):
    if self._cruise_cancel_state: 
      self._add_log(reason + " > Cancel state")
    elif enable > 0 and self._cancel_timer > 0 and cancel_timer >= 0:
      enable = 0
      self._add_log(reason + " > Canceled")
    else:
      if self.autoCruiseControl == 0 and enable != 0:
        enable = 0
        self._soft_hold_active = 0
        return
      if self.autoCruiseControl_cancel_timer > 0 and enable != 0:
        self._add_log(reason + " > timer Canceled")
        enable = 0
        self._soft_hold_active = 0
        return
      self._activate_cruise = enable
      self._cancel_timer = int(cancel_timer / 0.01)   # DT_CTRL: 0.01
      self._add_log(reason)

  def _check_safe_stop(self, CS, safe_distance = 3):
    v_ego = CS.vEgo
    decel_rate = 1.5
    d_stop_ego = (v_ego ** 2) / (2 * decel_rate)
    d_stop_rel = (self.v_rel ** 2) / (2 * decel_rate)

    d_final = self.d_rel - d_stop_ego - d_stop_rel

    if d_final >= safe_distance:
      return True, d_final
    else:
      return False, d_final

  def _update_cruise_state(self, CS, CC, v_cruise_kph):
    if not CC.enabled:
      self._pause_auto_speed_up = False
      self.carrot_cruise_active = False
      if self._brake_pressed_count == -1 and self._soft_hold_active > 0:
        self._soft_hold_active = 2
        self._cruise_control(1, -1, "Cruise on (soft hold)")
      # GM: autoResume
      elif self.params.get_bool("ActivateCruiseAfterBrake"):
        self.params.put_bool_nonblocking("ActivateCruiseAfterBrake", False)
        self._cruise_control(1, -1, "Cruise on (brake)")
      elif self.v_cruise_kph < self.v_ego_kph_set:
        self.v_cruise_kph = self.v_ego_kph_set

    if self._soft_hold_active > 0:
      pass

    if self._gas_tok and self.v_ego_kph_set >= self.autoGasTokSpeed:
      if not CC.enabled:
        self._cruise_control(1, -1, "Cruise on (gas tok)")
        if self.v_ego_kph_set > v_cruise_kph:
          v_cruise_kph = self.v_ego_kph_set
      else:
        v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)
    elif self._gas_pressed_count == -1:
      if 0 < self.d_rel < CS.vEgo * 0.8:
        if CS.vEgo < 1.0:
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (safe speed)")
        else:
          self._cruise_control(-1, 0, "Cruise off (lead car too close)")
      elif self.v_ego_kph_set < 30:
        self._cruise_control(-1, 0, "Cruise off (gas speed)")
      elif self.xState == 3:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(-1, 3, "Cruise off (traffic sign)")
      elif self.v_ego_kph_set >= self.autoGasTokSpeed and not CC.enabled:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas pressed)")
    elif self._brake_pressed_count == -1 and self._soft_hold_active == 0:
      if self._v_cruise_kph_at_brake > 0:
        # 松刹车时有刹车前时速，不要立即自动恢复，等待踩油门 1s 确认
        pass
      else:
        # 保持无状态时的原有行为
        if self.v_ego_kph_set > self.autoGasTokSpeed:
          v_cruise_kph = self.v_ego_kph_set
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (speed)")
        elif abs(CS.steeringAngleDeg) < 20:
          if self.xState in [3, 5]:
            if self.xState == 3:  # 减速中
              v_cruise_kph = self.v_ego_kph_set
            self._cruise_control(1, 0, "Cruise on (traffic sign)")
          elif 0 < self.d_rel < 20:
            self._cruise_control(1, -1 if self.v_ego_kph_set < 1 else 0, "Cruise on (lead car)")

    elif self._brake_pressed_count < 0 and self._gas_pressed_count < 0:
      if not CC.enabled:
        if self.d_rel > 0 and CS.vEgo > 0.02:
          safe_state, safe_dist = self._check_safe_stop(CS, 4)
          if abs(CS.steeringAngleDeg) > 70:
            pass
          elif not safe_state:
            self._cruise_control(1, -1, "Cruise on (fcw)")
          elif self.d_rel < self.cruiseOnDist:
            self._cruise_control(1, 0, "Cruise on (fcw dist)")
          else:
            self._add_log(f"leadCar d={self.d_rel:.1f},v={self.v_rel:.1f},{CS.vEgo:.1f}, {safe_dist:.1f}")
        if self.desiredSpeed < self.v_ego_kph_set:
          self._cruise_control(1, -1, "Cruise on (desired speed)")
        if self._cruise_ready:
          if self.xState in [3]:
            self._cruise_control(1, 0, "Cruise on (traffic sign)")
          elif self.d_rel > 0:
            self._cruise_control(1, 0, "Cruise on (lead car)")
      elif self._paddle_decel_active:
        if self.xState in [3]:
          self._paddle_decel_active = False
          v_cruise_kph = self.v_ego_kph_set
        elif self.d_rel > 0:
          self._paddle_decel_active = False
          v_cruise_kph = self.v_ego_kph_set


    # 踩油门持续 1.0 秒（100 帧）且有刹车前存储的时速，自动恢复纵向到原设定时速与限速之最大值
    if self._gas_pressed_count >= 100 and self._v_cruise_kph_at_brake > 0:
      road_limit = self.nRoadLimitSpeed
      if road_limit > 0:
        v_cruise_kph = max(self._v_cruise_kph_at_brake, road_limit, self._cruise_speed_min)
      else:
        v_cruise_kph = max(self._v_cruise_kph_at_brake, self._cruise_speed_min)
      self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas 1s resume)")
      self._v_cruise_kph_at_brake = 0

    if self._gas_pressed_count > self._gas_tok_timer:
      if CS.aEgo < -0.5:
        self._cruise_control(-1, 5.0, "Cruise off (gas pressed while braking)")
      if self.v_ego_kph_set > v_cruise_kph and self.autoGasSyncSpeed:
        v_cruise_kph = self.v_ego_kph_set

    if self._gas_pressed_count == 1 or CS.vEgo < 0.1:
      self._pause_auto_speed_up = False
      if self._gas_pressed_count == 1 and CS.vEgo < 0.1:
        self._cruise_control(-1, -1, "Cruise off (gasPressed)")
    elif self._brake_pressed_count == 1:
      self._pause_auto_speed_up = True
      # 踩刹车去掉纵向
      self._cruise_control(-1, -1, "Cruise off (brake pressed)")

    v_cruise_kph = self._auto_speed_up(v_cruise_kph)

    # =========================================================================
    # 【运动学前瞻性舒适跟车与应急避险减速控制算法（KPFE）】
    # =========================================================================
    if self.d_rel > 0 and CC.enabled:
      v_ego_kph = CS.vEgo * CV.MS_TO_KPH
      # 只在前方车辆速度比自车速度慢 20km/h 以上时，才触发前瞻限速调整
      if self.v_lead_kph > 0 and (v_ego_kph - self.v_lead_kph) > 20.0:
        if self._v_cruise_kph_pre_kpfe == 0:
          self._v_cruise_kph_pre_kpfe = v_cruise_kph
          
        # 1. 前车绝对时速 (m/s)
        v_lead = max(0.0, CS.vEgo + self.v_rel)
        
        # 2. 动态跟车安全缓冲距离 (基于前车速度的 1.4s 反应时间 + 6.0m 基础站立距离)
        d_buffer = max(6.0, v_lead * 1.4 + 4.0)
        
        # 3. 舒适减速度定义 (正常跟车时前瞻减速度设为 1.0 m/s^2，非常平缓)
        a_comfortable = 1.0
        
        # 4. 基于运动学恒定减速轨迹计算安全极限速度
        if self.d_rel >= d_buffer:
          # 在安全缓冲之外，计算以 a_comfortable 舒适减速到前车速度的物理限速
          safe_d = self.d_rel - d_buffer
          v_limit_ms = math.sqrt(v_lead ** 2 + 2.0 * a_comfortable * safe_d)
        else:
          # 【应急防加塞/防急刹机制】：已低于安全缓冲距离
          # 此时限制车速直接低于前车速度，按偏差比例拉低限制，产生明确的刹车制动力以拉开距离
          dist_error = d_buffer - self.d_rel
          # 每短于缓冲 1 米，速度限制就比前车慢 0.5 m/s (~1.8 km/h)
          v_limit_ms = max(0.0, v_lead - 0.5 * dist_error)
        
        # 5. 正在接近前车 (v_rel < 0) 时的平滑惯性微调：让减速动作更加前置
        if self.v_rel < 0.0:
          # 根据接近率 (v_rel) 施加阻尼，接近越快，车速提前压得越低
          v_limit_ms += self.v_rel * 0.4
        
        # 6. 将限制速度转换为 km/h，并设定 20 km/h 最低安全阈值，防止在行进中被限速为零
        v_visual_limit = max(20.0, v_limit_ms * CV.MS_TO_KPH)
        
        # 7. 应用限制
        if v_visual_limit < v_cruise_kph:
          v_cruise_kph = v_visual_limit
          if self.frame % 1000 == 0:
            print(f"[VISUAL_ACC] KPFE Active - dRel: {self.d_rel:.1f}m, vRel: {self.v_rel:.1f}m/s, Limit: {v_cruise_kph:.1f}kph")

      # 8. 距离补偿 EMA 滤波算法：智能平滑跟随前车加速
      elif self.v_lead_kph > 0:
        target_max = max(100.0, self._v_cruise_manual_max)
        
        # 1. 动态距离补偿
        # 设定前车 1.5 秒时距为安全距离，加上 5 米的基础车距
        v_lead_ms = self.v_lead_kph / 3.6
        d_safe = v_lead_ms * 1.5 + 5.0
        
        # 如果我们离前车比安全距离远，允许目标速度快得多去追赶（Kp=0.2 即每远1米，快0.2km/h）
        speed_gain = max(2.0, (self.d_rel - d_safe) * 0.2) 
        
        # 2. 计算原始追赶目标
        raw_target = min(self.v_lead_kph + speed_gain, target_max)
        
        # 3. EMA 低通滤波应用
        if raw_target > v_cruise_kph and v_cruise_kph < target_max:
          # alpha 平滑系数：0.02 代表每帧只逼近 2%，实现 S 型丝滑提速
          alpha = 0.02
          v_cruise_kph = v_cruise_kph * (1.0 - alpha) + raw_target * alpha
          v_cruise_kph = round(v_cruise_kph, 1) # 保持精度
          
          if self.frame % 1000 == 0:
            print(f"[VISUAL_ACC] EMA Follow - Target: {v_cruise_kph:.1f}kph (Raw: {raw_target:.1f}, Max: {target_max})")
            
        self._v_cruise_kph_pre_kpfe = 0

    return v_cruise_kph

  def _prepare_brake_gas(self, CS, CC):
    if CS.gasPressed:
      self._paddle_decel_active = False
      self.carrot_cruise_active = False
      self._gas_pressed_count = max(1, self._gas_pressed_count + 1)
      self._gas_pressed_count_last = self._gas_pressed_count
      self._gas_pressed_value = max(CS.gas, self._gas_pressed_value) if self._gas_pressed_count > 1 else CS.gas
      self._gas_tok = False
      self._soft_hold_active = 0
    else:
      self._gas_tok = True if 0 < self._gas_pressed_count < self._gas_tok_timer else False
      self._gas_pressed_count = min(-1, self._gas_pressed_count - 1)
      if self._gas_pressed_count < -1:
        self._gas_pressed_count_last = 0
        self._gas_tok = False

    if CS.brakePressed:
      self._cruise_ready = False
      self._paddle_decel_active = False
      self.carrot_cruise_active = False
      self._brake_pressed_count = max(1, self._brake_pressed_count + 1)
      if self._brake_pressed_count == 1 and self.enabled_last:
        self._v_cruise_kph_at_brake = self.v_cruise_kph
        self._add_log(f"{self.v_cruise_kph} Cruise speed at brake")
      self._soft_hold_count = self._soft_hold_count + 1 if CS.vEgo < 0.1 and CS.gearShifter == GearShifter.drive else 0
      if self.autoCruiseControl == 0 or self.CP.pcmCruise:
        self._soft_hold_active = 0
      else:
        self._soft_hold_active = 1 if self._soft_hold_count > 60 else 0
    else:
      self._soft_hold_count = 0
      self._brake_pressed_count = min(-1, self._brake_pressed_count - 1)
