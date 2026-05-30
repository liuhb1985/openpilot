import fcntl
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
import numpy as np
from datetime import datetime

from ftplib import FTP
from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.system.hardware import PC, TICI
from openpilot.selfdrive.navd.helpers import Coordinate
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.gps import get_gps_location_service
from openpilot.selfdrive.carrot.config import UnifiedParams

nav_type_mapping = {
  12: ("turn", "left", 1),
  16: ("turn", "sharp left", 1),
  1000: ("turn", "slight left", 1),
  1001: ("turn", "slight right", 2),
  1002: ("fork", "slight left", 3),
  1003: ("fork", "slight right", 4),
  1006: ("off ramp", "left", 3),
  1007: ("off ramp", "right", 4),
  13: ("turn", "right", 2),
  19: ("turn", "sharp right", 2),
  102: ("off ramp", "slight left", 3),
  105: ("off ramp", "slight left", 3),
  112: ("off ramp", "slight left", 3),
  115: ("off ramp", "slight left", 3),
  101: ("off ramp", "slight right", 4),
  104: ("off ramp", "slight right", 4),
  111: ("off ramp", "slight right", 4),
  114: ("off ramp", "slight right", 4),
  7: ("fork", "left", 3),
  44: ("fork", "left", 3),
  17: ("fork", "left", 3),
  75: ("fork", "left", 3),
  76: ("fork", "left", 3),
  118: ("fork", "left", 3),
  6: ("fork", "right", 4),
  43: ("fork", "right", 4),
  73: ("fork", "right", 4),
  74: ("fork", "right", 4),
  123: ("fork", "right", 4),
  124: ("fork", "right", 4),
  117: ("fork", "right", 4),
  131: ("rotary", "slight right", 5),
  132: ("rotary", "slight right", 5),
  140: ("rotary", "slight left", 5),
  141: ("rotary", "slight left", 5),
  133: ("rotary", "right", 5),
  134: ("rotary", "sharp right", 5),
  135: ("rotary", "sharp right", 5),
  136: ("rotary", "sharp left", 5),
  137: ("rotary", "sharp left", 5),
  138: ("rotary", "sharp left", 5),
  139: ("rotary", "left", 5),
  142: ("rotary", "straight", 5),
  14: ("turn", "uturn", 5),
  201: ("arrive", "straight", 5),
  51: ("notification", "straight", None),
  52: ("notification", "straight", None),
  53: ("notification", "straight", None),
  54: ("notification", "straight", None),
  55: ("notification", "straight", None),
  153: ("", "", 6),  #TG
  154: ("", "", 6),  #TG
  249: ("", "", 6)   #TG
}

import collections
class CarrotServ:
  def __init__(self):
    self.params = UnifiedParams()
    self.params_memory = Params("/dev/shm/params")

    self.nRoadLimitSpeed = 30
    self.nRoadLimitSpeed_last = 30
    self.nRoadLimitSpeed_counter = 0

    self.active_carrot = 0     ## 1: CarrotMan 激活, 2: SDI 激活 , 3: 减速控制激活, 4: 区间测速激活, 5: 减速带激活, 6: 限速激活
    self.active_count = 0
    self.active_sdi_count = 0
    self.active_sdi_count_max = 200 # 20 秒

    self.active_kisa_count = 0

    self.nSdiType = -1
    self.nSdiSpeedLimit = 0
    self.nSdiSection = 0
    self.nSdiDist = 0
    self.nSdiBlockType = -1
    self.nSdiBlockSpeed = 0
    self.nSdiBlockDist = 0

    self.nTBTDist = 0
    self.nTBTTurnType = -1
    self.szTBTMainText = ""
    self.szNearDirName = ""
    self.szFarDirName = ""
    self.nTBTNextRoadWidth = 0

    self.nTBTDistNext = 0
    self.nTBTTurnTypeNext = -1
    self.szTBTMainTextNext = ""

    self.xTurnInfo = -1
    self.xDistToTurn = 0
    self.xDistToTurnNav = 0
    self.xDistToTurnMax = 0
    self.xDistToTurnMaxCnt = 0 #最大转弯引导距离的次数
    self.xDistToTurnNavLast = 0
    self.xTurnInfoNext = -1
    self.xDistToTurnNext = 0

    self.nGoPosDist = 0
    self.nGoPosTime = 0
    self.szPosRoadName = ""
    self.nSdiPlusType = -1
    self.nSdiPlusSpeedLimit = 0
    self.nSdiPlusDist = 0
    self.nSdiPlusBlockType = -1
    self.nSdiPlusBlockSpeed = 0
    self.nSdiPlusBlockDist = 0

    self.goalPosX = 0.0
    self.goalPosY = 0.0
    self.szGoalName = ""
    self.vpPosPointLatNavi = 0.0
    self.vpPosPointLonNavi = 0.0
    self.vpPosPointLat = 0.0
    self.vpPosPointLon = 0.0
    self.roadcate = 8

    self.nPosSpeed = 0.0
    self.nPosAngle = 0.0
    self.nPosAnglePhone = 0.0

    self.diff_angle_count = 0
    self.last_calculate_gps_time = 0
    self.last_update_gps_time = 0
    self.last_update_gps_time_phone = 0
    self.last_update_gps_time_navi = 0
    self.bearing_offset = 0.0
    self.bearing_measured = 0.0
    self.bearing = 0.0
    self.gps_valid = False

    self.phone_gps_accuracy = 0.0
    self.gps_accuracy_device = 0.0
    self.phone_latitude = 0.0
    self.phone_longitude = 0.0
    self.phone_gps_frame = 0

    self.totalDistance = 0
    self.xSpdLimit = 0
    self.xSpdDist = 0
    self.xSpdType = -1


    self.navType, self.navModifier = "invalid", ""
    self.navTypeNext, self.navModifierNext = "invalid", ""

    self.carrotIndex = 0
    self.carrotCmdIndex = 0
    self.carrotCmd = ""
    self.carrotArg = ""
    self.carrotCmdIndex_last = 0

    self.traffic_light_q = collections.deque(maxlen=int(2.0/0.1))  # 2 秒
    self.traffic_light_count = -1
    self.traffic_state = 0

    self.left_spd_sec = 0
    self.left_tbt_sec = 0
    self.left_sec = 100
    self.max_left_sec = 100
    self.carrot_left_sec = 100
    self.sdi_inform = False


    self.atc_paused = False
    self.atc_activate_count = 0
    self.gas_override_speed = 0
    self.gas_pressed_state = False
    self.source_last = "none"

    self.debugText = ""

    # 默认语言，稍后在 update_params 中从 Params 读取覆盖，
    # 规则：main_ko -> 韩语；main_zh-CHS -> 中文；其他 -> 英文
    self.lang = "en"

    self.update_params()

  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedCtrlMode = self.params.get_int("AutoNaviSpeedCtrlMode")
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviCountDownMode = self.params.get_int("AutoNaviCountDownMode")
    self.turnSpeedControlMode = self.params.get_int("TurnSpeedControlMode")
    self.mapTurnSpeedFactor = self.params.get_float("MapTurnSpeedFactor") * 0.01

    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    self.autoCurveSpeedLowerLimit = int(self.params.get("AutoCurveSpeedLowerLimit") or 30)
    self.is_metric = self.params.get_bool("IsMetric")
    self.autoRoadSpeedLimitOffset = self.params.get_int("AutoRoadSpeedLimitOffset")

    self.autoTurnDistOffset = self.params.get_int("AutoTurnDistOffset")
    self.autoForkDistOffset = self.params.get_int("AutoForkDistOffset")
    self.autoDoForkBlinkerDist = self.params.get_int("AutoDoForkBlinkerDist")
    self.autoDoForkNavDist = self.params.get_int("AutoDoForkNavDist")
    self.autoForkDistOffsetH = self.params.get_int("AutoForkDistOffsetH")
    self.autoDoForkDecalDistH = self.params.get_int("AutoDoForkDecalDistH")
    self.autoDoForkDecalDist = self.params.get_int("AutoDoForkDecalDist")
    self.autoDoForkBlinkerDistH = self.params.get_int("AutoDoForkBlinkerDistH")
    self.autoDoForkNavDistH = self.params.get_int("AutoDoForkNavDistH")
    self.autoUpRoadLimit = self.params.get_int("AutoUpRoadLimit")
    self.autoUpRoadLimit40KMH = self.params.get_int("AutoUpRoadLimit40KMH")
    self.autoUpHighwayRoadLimit = self.params.get_int("AutoUpHighwayRoadLimit")
    self.autoUpHighwayRoadLimit40KMH = self.params.get_int("AutoUpHighwayRoadLimit40KMH")
    self.roadType = self.params.get_int("RoadType")
    self.autoForkDecalRateH = float(self.params.get_int("AutoForkDecalRateH")) * 0.01
    self.autoForkSpeedMinH = self.params.get_int("AutoForkSpeedMinH")
    self.autoKeepForkSpeedH = self.params.get_int("AutoKeepForkSpeedH")
    self.autoForkDecalRate = float(self.params.get_int("AutoForkDecalRate")) * 0.01
    self.autoForkSpeedMin = self.params.get_int("AutoForkSpeedMin")
    self.autoKeepForkSpeed = self.params.get_int("AutoKeepForkSpeed")
    self.showDebugLog = self.params.get_int("ShowDebugLog")

    if self.roadType >= 0: #手动设置道路类型
      self.roadcate = self.roadType

    # 读取语言设置：优先使用 LanguageSetting，与 UI 保持一致；回退读取可能存在的 "lang"
    try:
      lang_val = self.params.get('LanguageSetting', encoding='utf8') or self.params.get('lang', encoding='utf8')
    except Exception:
      lang_val = None
    if isinstance(lang_val, bytes):
      try:
        lang_val = lang_val.decode('utf8')
      except Exception:
        lang_val = None
    if lang_val == "main_ko":
      self.lang = "ko"
    elif lang_val == "main_zh-CHS":
      self.lang = "zh"
    else:
      self.lang = "en"


  def _update_cmd(self):
    if self.carrotCmdIndex != self.carrotCmdIndex_last:
      self.carrotCmdIndex_last = self.carrotCmdIndex
      command_handlers = {
        "DETECT": self._handle_detect_command,
      }

      handler = command_handlers.get(self.carrotCmd)
      if handler:
        handler(self.carrotArg)

    self.traffic_light_q.append((-1, -1, "none", 0.0))
    self.traffic_light_count -= 1
    if self.traffic_light_count < 0:
      self.traffic_light_count = -1
      self.traffic_state = 0

  def _handle_detect_command(self, xArg):
    elements = [e.strip() for e in xArg.split(',')]
    if len(elements) >= 4:
      try:
        state = elements[0]
        value1 = float(elements[1])
        value2 = float(elements[2])
        value3 = float(elements[3])
        self.traffic_light(value1, value2, state, value3)
        self.traffic_light_count = int(0.5 / 0.1)
      except ValueError:
        pass

  def traffic_light(self, x, y, color, cnf):
    traffic_red = 0
    traffic_green = 0
    traffic_left = 0
    traffic_red_trig = 0
    traffic_green_trig = 0
    traffic_left_trig = 0
    for pdata in self.traffic_light_q:
      px, py, pcolor,pcnf = pdata
      if abs(x - px) < 0.2 and abs(y - py) < 0.2:
        if pcolor in ["Green Light", "Left turn"]:
          if color in ["Red Light", "Yellow Light"]:
            traffic_red_trig += cnf
            traffic_red += cnf
          elif color in ["Green Light", "Left turn"]:
            traffic_green += cnf
        elif pcolor in ["Red Light", "Yellow Light"]:
          if color in ["Green Light"]: #, "Left turn"]:
            traffic_green_trig += cnf
            traffic_green += cnf
          elif color in ["Left turn"]:
            traffic_left_trig += cnf
            traffic_left += cnf
          elif color in ["Red Light", "Yellow Light"]:
            traffic_red += cnf

    #print(self.traffic_light_q)
    if traffic_red_trig > 0:
      self.traffic_state = 1
      #self._add_log("红灯触发")
      #print("Red light triggered")
    elif traffic_green_trig > 0 and traffic_green > traffic_red:  # 如果绿灯置信度高于红灯则启动... 防止误判
      self.traffic_state = 2
      #self._add_log("绿灯触发")
      #print("Green light triggered")
    elif traffic_left_trig > 0:
      self.traffic_state = 3
    elif traffic_red > 0:
      self.traffic_state = 1
      #self._add_log("持续红灯")
      #print("Red light continued")
    elif traffic_green > 0:
      self.traffic_state = 2
      #self._add_log("持续绿灯")
      #print("Green light continued")
    else:
      self.traffic_state = 0
      #print("TrafficLight none")

    self.traffic_light_q.append((x,y,color,cnf))


  def calculate_current_speed(self, left_dist, safe_speed_kph, safe_time, safe_decel_rate):
    safe_speed = safe_speed_kph / 3.6
    safe_dist = safe_speed * safe_time
    decel_dist = left_dist - safe_dist

    if decel_dist <= 0:
      return safe_speed_kph

    # v_i^2 = v_f^2 + 2ad
    temp = safe_speed**2 + 2 * safe_decel_rate * decel_dist  # 公式应用减速控制

    if temp < 0:
      speed_mps = safe_speed
    else:
      speed_mps = math.sqrt(temp)
    return max(safe_speed_kph, min(250, speed_mps * 3.6))

  def _update_tbt(self):
    #xTurnInfo : 1: 左转, 2: 右转, 3: 左变道, 4: 右变道, 5: 环岛, 6: 收费站, 7: 到达或掉头
    turn_type_mapping = {
      12: ("turn", "left", 1),
      16: ("turn", "sharp left", 1),
      13: ("turn", "right", 2),
      19: ("turn", "sharp right", 2),
      102: ("off ramp", "slight left", 3),
      105: ("off ramp", "slight left", 3),
      112: ("off ramp", "slight left", 3),
      115: ("off ramp", "slight left", 3),
      101: ("off ramp", "slight right", 4),
      104: ("off ramp", "slight right", 4),
      111: ("off ramp", "slight right", 4),
      114: ("off ramp", "slight right", 4),
      7: ("fork", "left", 3),
      44: ("fork", "left", 3),
      17: ("fork", "left", 3),
      75: ("fork", "left", 3),
      76: ("fork", "left", 3),
      118: ("fork", "left", 3),
      6: ("fork", "right", 4),
      43: ("fork", "right", 4),
      73: ("fork", "right", 4),
      74: ("fork", "right", 4),
      123: ("fork", "right", 4),
      124: ("fork", "right", 4),
      117: ("fork", "right", 4),
      131: ("rotary", "slight right", 5),
      132: ("rotary", "slight right", 5),
      140: ("rotary", "slight left", 5),
      141: ("rotary", "slight left", 5),
      133: ("rotary", "right", 5),
      134: ("rotary", "sharp right", 5),
      135: ("rotary", "sharp right", 5),
      136: ("rotary", "sharp left", 5),
      137: ("rotary", "sharp left", 5),
      138: ("rotary", "sharp left", 5),
      139: ("rotary", "left", 5),
      142: ("rotary", "straight", 5),
      14: ("turn", "uturn", 7),
      201: ("arrive", "straight", 8),
      51: ("notification", "straight", 0),
      52: ("notification", "straight", 0),
      53: ("notification", "straight", 0),
      54: ("notification", "straight", 0),
      55: ("notification", "straight", 0),
      153: ("", "", 6),  #收费站 (TG)
      154: ("", "", 6),  #收费站 (TG)
      249: ("", "", 6)   #收费站 (TG)
    }

    if self.nTBTTurnType in turn_type_mapping:
      self.navType, self.navModifier, self.xTurnInfo = turn_type_mapping[self.nTBTTurnType]
    else:
      self.navType, self.navModifier, self.xTurnInfo = "invalid", "", -1

    if self.nTBTTurnTypeNext in turn_type_mapping:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = turn_type_mapping[self.nTBTTurnTypeNext]
    else:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1

    if self.nTBTDist > 0 and self.xTurnInfo > 0:
      self.xDistToTurnNav = self.nTBTDist
      self.xDistToTurn = self.xDistToTurnNav
    if self.nTBTDistNext > 0 and self.xTurnInfoNext > 0:
      self.xDistToTurnNext = self.nTBTDistNext + self.nTBTDist

  def _get_sdi_descr(self, nSdiType):
    # 多语言映射：ko（韩语），zh（简体中文），en（英文）。
    sdi_zh = {
        0: "信号与超速",
        1: "超速 (固定式)",
        2: "区间测速开始",
        3: "区间测速结束",
        4: "区间测速中",
        5: "闯绿灯/尾随违法抓拍",
        6: "闯红灯抓拍",
        7: "超速 (移动式)",
        8: "固定式超速危险区域 (箱式)",
        9: "公交专用车道",
        10: "可变车道执法",
        11: "路肩监控点",
        12: "禁止切入",
        13: "交通信息采集点",
        14: "安防 CCTV",
        15: "超速危险路段",
        16: "超限载重执法",
        17: "违停执法点",
        18: "单行道",
        19: "铁路道口",
        20: "儿童保护区 (学校区域开始)",
        21: "儿童保护区 (学校区域结束)",
        22: "减速带",
        23: "加气站 (LPG)",
        24: "隧道区域",
        25: "服务区",
        26: "收费站",
        27: "雾天多发区域",
        28: "有害物质区域",
        29: "事故多发",
        30: "急弯区域",
        31: "急弯路段 1",
        32: "陡坡路段",
        33: "野生动物出没区域",
        34: "右侧视野不良",
        35: "视野不良点",
        36: "左侧视野不良",
        37: "信号违章多发路段",
        38: "超速行驶多发路段",
        39: "交通拥堵区域",
        40: "分道选择点",
        41: "乱穿马路事故多发点",
        42: "路肩事故多发点",
        43: "超速事故多发点",
        44: "疲劳驾驶事故多发点",
        45: "事故多发点",
        46: "行人事故多发点",
        47: "车辆防盗重点区域",
        48: "落石注意区域",
        49: "结冰注意区域",
        50: "瓶颈点",
        51: "合流道路",
        52: "坠落注意区域",
        53: "地下通道区域",
        54: "住宅密集区 (交通缓行区)",
        55: "互通立交 (IC)",
        56: "枢纽立交 (JCT)",
        57: "服务区 (可加 LPG)",
        58: "桥梁",
        59: "制动失效事故多发点",
        60: "越线行驶事故多发点",
        61: "违章通行事故多发点",
        62: "目的地对面导向",
        63: "疲劳驾驶休息区",
        64: "老旧柴油车限行",
        65: "隧道内禁止变道执法",
        66: "",
    }

    sdi_en = {
        0: "Signal speed enforcement",
        1: "Speed camera (fixed)",
        2: "Section control start",
        3: "Section control end",
        4: "Under section control",
        5: "Block-the-box camera",
        6: "Signal violation enforcement",
        7: "Speed camera (mobile)",
        8: "Fixed speed camera zone (box)",
        9: "Bus-only lane zone",
        10: "Reversible/variable lane enforcement",
        11: "Shoulder surveillance point",
        12: "No cut-in",
        13: "Traffic data collection point",
        14: "Security CCTV",
        15: "Overloaded vehicle risk zone",
        16: "Improper loading enforcement",
        17: "Parking enforcement point",
        18: "One-way road",
        19: "Railroad crossing",
        20: "School zone start",
        21: "School zone end",
        22: "Speed bump",
        23: "LPG station",
        24: "Tunnel section",
        25: "Rest area",
        26: "Toll gate",
        27: "Fog caution area",
        28: "Hazardous materials area",
        29: "Accident-prone section",
        30: "Sharp curve area",
        31: "Sharp curve section 1",
        32: "Steep slope section",
        33: "Wild animal crossing area",
        34: "Poor visibility (right)",
        35: "Poor visibility",
        36: "Poor visibility (left)",
        37: "Frequent signal violations",
        38: "Frequent speeding",
        39: "Traffic congestion area",
        40: "Lane selection by direction",
        41: "Frequent jaywalking accidents",
        42: "Frequent shoulder accidents",
        43: "Frequent speeding accidents",
        44: "Frequent drowsy driving accidents",
        45: "Accident-prone spot",
        46: "Frequent pedestrian accidents",
        47: "Frequent vehicle theft",
        48: "Falling rock caution area",
        49: "Icy road caution area",
        50: "Bottleneck point",
        51: "Merging road",
        52: "Cliff/Drop caution area",
        53: "Underpass section",
        54: "Residential area (traffic calming)",
        55: "Interchange",
        56: "Junction",
        57: "Rest area (LPG available)",
        58: "Bridge",
        59: "Frequent brake failure accidents",
        60: "Center line invasion accidents",
        61: "Violation-of-passage accidents",
        62: "Destination on opposite side",
        63: "Drowsy rest area",
        64: "Old diesel control",
        65: "Lane change enforcement in tunnel",
        66: "",
    }

    sdi_zh = {
        0: "信号测速/闯灯拍照",
        1: "固定测速摄像头",
        2: "区间测速开始",
        3: "区间测速结束",
        4: "区间测速中",
        5: "路口压线摄像头",
        6: "闯红灯拍照",
        7: "流动测速摄像头",
        8: "测速拍照",
        9: "公交专用车道区间",
        10: "可变/潮汐车道拍照",
        11: "应急车道拍照",
        12: "禁止加塞",
        13: "交通信息采集点",
        14: "治安监控",
        15: "超载车辆风险区",
        16: "装载不当拍照",
        17: "违停拍照点",
        18: "单行道",
        19: "铁路道口",
        20: "学校区域开始",
        21: "学校区域结束",
        22: "减速带",
        23: "LPG加气站",
        24: "隧道区间",
        25: "服务区",
        26: "ETC计费拍照",
        27: "多雾路段",
        28: "危险品区域",
        29: "事故多发路段",
        30: "急弯路段",
        31: "急弯区段1",
        32: "陡坡路段",
        33: "野生动物出没路段",
        34: "右侧视野不良点",
        35: "视野不良点",
        36: "左侧视野不良点",
        37: "闯红灯多发",
        38: "超速多发",
        39: "交通拥堵区域",
        40: "按方向选择车道点",
        41: "行人乱穿马路多发处",
        42: "应急车道事故多发",
        43: "超速事故多发",
        44: "疲劳驾驶事故多发",
        45: "事故多发点",
        46: "行人事故多发点",
        47: "车辆盗窃多发点",
        48: "落石危险路段",
        49: "路面结冰危险",
        50: "瓶颈路段",
        51: "汇入道路",
        52: "坠落危险路段",
        53: "地下车道区间",
        54: "居民区（交通缓和）",
        55: "立交",
        56: "分岔点",
        57: "服务区（可加气）",
        58: "桥梁",
        59: "制动故障事故多发点",
        60: "越线事故多发点",
        61: "违法通行事故多发点",
        62: "目的地在对面",
        63: "瞌睡停车区",
        64: "老旧柴油车管制",
        65: "隧道内变道拍照",
        66: "",
    }

    sdi_map = sdi_en
    if self.lang == "ko":
      sdi_map = sdi_ko
    elif self.lang == "zh":
      sdi_map = sdi_zh

    return sdi_map.get(nSdiType, "")

  def _update_sdi(self):
    # sdiBlockType:
    # 1: startOSEPS: 区间测速开始
    # 2: inOSEPS: 区间测速中
    # 3: endOSEPS: 区间测速结束
    # 0: 不减速, 1: 测速摄像头, 2: +减速带, 3: +流动测速摄像头
    if self.nSdiType in [0,1,2,3,4,7,8, 75, 76] and self.nSdiSpeedLimit > 0 and self.autoNaviSpeedCtrlMode > 0:
      self.xSpdLimit = self.nSdiSpeedLimit * self.autoNaviSpeedSafetyFactor
      self.xSpdDist = self.nSdiDist
      self.xSpdType = self.nSdiType
      if self.nSdiBlockType in [2,3]:
        self.xSpdDist = self.nSdiBlockDist
        self.xSpdType = 4
      elif self.nSdiType == 7 and self.autoNaviSpeedCtrlMode < 3: # 流动测速
        self.xSpdLimit = self.xSpdDist = 0
    elif (self.nSdiPlusType == 22 or self.nSdiType == 22) and self.roadcate > 1 and self.autoNaviSpeedCtrlMode >= 2: # 减速带, roadcate:0,1 为高速
      self.xSpdLimit = self.autoNaviSpeedBumpSpeed
      self.xSpdDist = self.nSdiPlusDist if self.nSdiPlusType == 22 else self.nSdiDist
      self.xSpdType = 22
    else:
      self.xSpdLimit = 0
      self.xSpdType = -1
      self.xSpdDist = 0

  def _update_gps(self, v_ego, sm, gps_service):
    gps = sm[gps_service]
    if not sm.updated['carState'] or not sm.updated['carControl']:
      return self.nPosAngle
    CS = sm['carState']
    CC = sm['carControl']
    self.gps_valid = sm.updated[gps_service] and gps.hasFix

    now = time.monotonic()
    gps_updated_phone = (now - self.last_update_gps_time_phone) < 3
    gps_updated_navi = (now - self.last_update_gps_time_navi) < 3

    bearing = self.nPosAngle
    if gps_updated_phone:
      self.bearing_offset = 0.0
    elif self.gps_valid:
      bearing = self.nPosAngle = gps.bearingDeg
      if self.gps_valid:
        self.bearing_offset = 0.0
      elif self.active_carrot > 0:
        bearing = self.nPosAnglePhone
        self.bearing_offset = 0.0

    gpsDelayTimeAdjust = 0.0
    if gps_updated_navi:
      gpsDelayTimeAdjust = 0 

    external_gps_update_timedout = not (gps_updated_phone or gps_updated_navi)
    if self.gps_valid and external_gps_update_timedout:    # 内部 GPS 运行且无 CarrotMan 信号
      self.vpPosPointLatNavi = gps.latitude
      self.vpPosPointLonNavi = gps.longitude
      self.last_calculate_gps_time = now 
    elif gps_updated_navi:  # 接收到 Carrot 导航 GPS 信号
      if abs(self.bearing_measured - bearing) < 0.1:
          self.diff_angle_count += 1
      else:
          self.diff_angle_count = 0
      self.bearing_measured = bearing

      if self.diff_angle_count > 5: # 仅在转向角度几乎无变化时更新
        diff_angle = (self.nPosAngle - bearing) % 360
        if diff_angle > 180:
          diff_angle -= 360
        self.bearing_offset = self.bearing_offset * 0.9 + diff_angle * 0.1

    bearing_calculated = (bearing + self.bearing_offset) % 360

    dt = now - self.last_calculate_gps_time
    if dt > 5.0:
      self.vpPosPointLat, self.vpPosPointLon = 0.0, 0.0
    elif dt == 0:
      self.vpPosPointLat, self.vpPosPointLon = self.vpPosPointLatNavi, self.vpPosPointLonNavi
    else:
      self.vpPosPointLat, self.vpPosPointLon = self.estimate_position(float(self.vpPosPointLatNavi), float(self.vpPosPointLonNavi), v_ego, bearing_calculated, dt + gpsDelayTimeAdjust)

    return float(bearing_calculated)


  def estimate_position(self, lat, lon, speed, angle, dt):
    R = 6371000
    angle_rad = math.radians(angle)
    delta_d = speed * dt
    delta_lat = delta_d * math.cos(angle_rad) / R
    new_lat = lat + math.degrees(delta_lat)
    delta_lon = delta_d * math.sin(angle_rad) / (R * math.cos(math.radians(lat)))
    new_lon = lon + math.degrees(delta_lon)

    return new_lat, new_lon

  def update_auto_turn(self, v_ego_kph, sm, x_turn_info, x_dist_to_turn, check_steer=False):
    turn_speed = self.autoTurnControlSpeedTurn
    fork_speed = self.nRoadLimitSpeed
    stop_speed = 1
    turn_dist_for_speed = self.autoTurnControlTurnEnd * turn_speed / 3.6 # 5
    fork_dist_for_speed = self.autoTurnControlTurnEnd * fork_speed / 3.6 # 5
    stop_dist_for_speed = 5
    if self.roadcate > 1:
      fork_dist_offset = self.autoForkDistOffset
      # [Task 21] 导航变道策略：统一高速路口变道基准点为 500m
      start_fork_dist = np.interp(self.nRoadLimitSpeed, [30, 50, 100, 120], [160, 200, 450, 500]) + fork_dist_offset
      do_fork_dist = fork_dist_for_speed + self.autoDoForkBlinkerDist
      do_speed_decal_dist = fork_dist_for_speed + self.autoDoForkDecalDist
      auto_decel_rate = self.autoForkDecalRate
      decel_speed_min = self.autoForkSpeedMin
      do_fork_nav_dist = self.autoDoForkNavDist
      fork_speed_keep_time = self.autoKeepForkSpeed
    else:
      fork_dist_offset = self.autoForkDistOffsetH
      start_fork_dist = np.interp(self.nRoadLimitSpeed, [30, 50, 100], [160, 200, 350]) + fork_dist_offset
      do_fork_dist = fork_dist_for_speed + self.autoDoForkBlinkerDistH
      do_speed_decal_dist = fork_dist_for_speed + self.autoDoForkDecalDistH
      auto_decel_rate = self.autoForkDecalRateH
      decel_speed_min = self.autoForkSpeedMinH
      do_fork_nav_dist = self.autoDoForkNavDistH
      fork_speed_keep_time = self.autoKeepForkSpeedH

    #对两个fork距离进行限制
    if do_fork_nav_dist > 0:
      do_fork_dist = max(do_fork_dist, do_fork_nav_dist)

    #对变道和转弯距离作一个限制，不能比最大路口距离的80%还大
    max_dist = self.xDistToTurnMax*0.8
    if do_fork_dist > max_dist:
      do_fork_dist = max_dist
    if start_fork_dist > max_dist:
      start_fork_dist = max_dist

    start_turn_dist = np.interp(self.nTBTNextRoadWidth, [5, 10], [43, 60]) + self.autoTurnDistOffset
    turn_info_mapping = {
        1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        2: {"type": "turn right", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
        5: {"type": "straight", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_turn_dist},
        3: {"type": "fork left", "speed": fork_speed, "dist": do_fork_dist, "start": start_fork_dist},
        4: {"type": "fork right", "speed": fork_speed, "dist": do_fork_dist, "start": start_fork_dist},
        6: {"type": "straight", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
        7: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
        8: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0, "start": 1000}

    mapping = turn_info_mapping.get(x_turn_info, default_mapping)

    atc_type = mapping["type"]
    atc_speed = mapping["speed"]
    atc_dist = mapping["dist"] #这个距离主要是用来提前减速的距离
    atc_start_dist = mapping["start"]
    atc_type_org = atc_type
    atc_speed_org = atc_speed
    atc_bsd_adjust_enable = False
    atc_speed_up = 0
    atc_control = False
    delta_v = None
    speed_up_enable = True
    decel_priority = False

    #导航给出在转弯距离大于开始转弯距离时，进入准备阶段
    if x_dist_to_turn > atc_start_dist:
      atc_type += " prepare"
      if check_steer:
        self.atc_activate_count = min(0, self.atc_activate_count - 1)
    else:
      if check_steer:
        self.atc_activate_count = max(0, self.atc_activate_count + 1)

      if atc_type in ["turn left", "turn right"]:
        if x_dist_to_turn > start_turn_dist:
          atc_type = "atc left" if "left" in atc_type else "atc right" #类型为atc left/right只是进入转弯准备状态，并不是真的在执行转弯
          atc_bsd_adjust_enable = True
          decel_priority = True
        else:
          atc_control = True
      elif atc_type in ["fork left", "fork right"]: #说明x_dist_to_turn>do_fork_dist并且说明x_dist_to_turn <=atc_start_dist
        #atc_dist = do_speed_decal_dist #替换减速距离
        if (fork_dist_offset > 0) and (500 > x_dist_to_turn > do_fork_dist): # 修正：增加 500m 硬性卡点，解决 2km 过早变道问题
          atc_type = "atc left" if "left" in atc_type else "atc right"
          atc_bsd_adjust_enable = True
          if x_dist_to_turn < 500: #小于500米时优先减速
            decel_priority = True
        elif (do_fork_nav_dist > 0) and (x_dist_to_turn <= do_fork_nav_dist): #设置了导航距离控制转弯后，如果距离小于设置值是立即变道
          atc_type += " now"
        if x_dist_to_turn < do_speed_decal_dist and (0 <= self.roadcate <= 1 or "right" in atc_type): #距离路口的距离小于设定值时要开始减速了，因为到匝道口前nRoadLimitSpeed其实没有变，所以只能用这种方法进行减速
          atc_bsd_adjust_enable = False
          if auto_decel_rate > 0 and check_steer: #设置了减速比率
            if atc_speed > decel_speed_min: #只有车速大于最小设定速度时才允许降速
              atc_speed = max(decel_speed_min, atc_speed*auto_decel_rate*0.8) # 强化0.8倍的进匝道减速要求
              self.atc_speed_decal = atc_speed #保存进匝道减速的目标速度
              self.fork_speed_keep_time = int(fork_speed_keep_time/0.1) #重置保持时间

    if check_steer:
      if 0 <= x_dist_to_turn < atc_start_dist and atc_type in ["fork left", "fork right"]:
        if not self.atc_paused:
          steering_pressed = sm["carState"].steeringPressed
          steering_torque = sm["carState"].steeringTorque
          if steering_pressed and steering_torque < 0 and atc_type in ["fork left", "atc left"]:
            self.atc_paused = True
          elif steering_pressed and steering_torque > 0 and atc_type in ["fork right", "atc right"]:
            self.atc_paused = True
      else:
        self.atc_paused = False

      if self.atc_paused:
        atc_type += " canceled"

    atc_desired = 250
    if atc_speed > 0 and x_dist_to_turn > 0:
      decel = self.autoNaviSpeedDecelRate
      safe_sec = 2.0
      atc_desired = min(atc_desired, self.calculate_current_speed(x_dist_to_turn - atc_dist, atc_speed, safe_sec, decel))

    return atc_desired, atc_type, atc_speed, atc_dist, atc_speed_up

  def update_nav_instruction(self, sm):
    if sm.alive['navInstruction'] and sm.valid['navInstruction']:
      msg_nav = sm['navInstruction']

      self.nGoPosDist = int(msg_nav.distanceRemaining)
      self.nGoPosTime = int(msg_nav.timeRemaining)
      if self.active_kisa_count <= 0 and msg_nav.speedLimit > 0:
        self.nRoadLimitSpeed = max(30, round(msg_nav.speedLimit * CV.MS_TO_KPH))
      self.xDistToTurn = int(msg_nav.maneuverDistance)
      self.szTBTMainText = msg_nav.maneuverPrimaryText
      self.xTurnInfo = -1
      for key, value in nav_type_mapping.items():
        if value[0] == msg_nav.maneuverType and value[1] == msg_nav.maneuverModifier:
          self.xTurnInfo = value[2]
          break

      self.debugText = f"{self.nRoadLimitSpeed if self.is_metric else self.nRoadLimitSpeed * CV.KPH_TO_MPH:.0f},{msg_nav.maneuverType},{msg_nav.maneuverModifier} "

  def update_kisa(self, data):
    self.active_kisa_count = 100
    if "kisawazecurrentspd" in data:
      pass
    if "kisawazeroadspdlimit" in data:
      road_limit_speed = data["kisawazeroadspdlimit"]
      if road_limit_speed > 0:
        print(f"waze 道路限速: {road_limit_speed} km/h")
        if not self.is_metric:
          road_limit_speed *= CV.MPH_TO_KPH
        self.nRoadLimitSpeed = road_limit_speed
    if "kisawazealert" in data:
      pass
    if "kisawazeendalert" in data:
      pass
    if "kisawazeroadname" in data:
      print(f"waze 道路名称: {data['kisawazeroadname']}")
      self.szPosRoadName = data["kisawazeroadname"]
    if "kisawazereportid" in data and "kisawazealertdist" in data:
      id_str = data["kisawazereportid"]
      dist_str = data["kisawazealertdist"].lower()
      import re
      match = re.search(r'(\d+)', dist_str)
      distance = int(match.group(1)) if match else 0
      if not self.is_metric:
        distance = int(distance * 0.3048)
      print(f"{id_str}: {distance} m")
      xSpdType = -1
      if 'camera' in id_str:
        xSpdType = 101    # 101: waze 测速摄像头, 100: 警察
      elif 'police' in id_str:
        xSpdType = 100

      if xSpdType >= 0:
        offset = 5 if self.is_metric else 5 * CV.MPH_TO_KPH
        self.xSpdLimit = self.nRoadLimitSpeed + offset

        self.xSpdDist = distance
        self.xSpdType =xSpdType

  def update_navi(self, remote_ip, sm, pm, vturn_speed, coords, distances, route_speed, gps_service, atc_curvature=0.0):

    self.debugText = ""
    self.update_params()
    if sm.alive['carState'] and sm.alive['selfdriveState']:
      CS = sm['carState']
      v_ego = CS.vEgo
      v_ego_kph = v_ego * 3.6
      distanceTraveled = sm['selfdriveState'].distanceTraveled
      delta_dist = distanceTraveled - self.totalDistance
      self.totalDistance = distanceTraveled
      if CS.speedLimit > 0:
        self.nRoadLimitSpeed = CS.speedLimit * 3.6  # m/s -> km/h
    else:
      v_ego = v_ego_kph = 0
      delta_dist = 0
      CS = None

    road_speed_limit_changed = True if self.nRoadLimitSpeed != self.nRoadLimitSpeed_last else False
    self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
    self.bearing = self._update_gps(v_ego, sm, gps_service)

    self.xSpdDist = max(self.xSpdDist - delta_dist, -1000)
    self.xDistToTurnNav = self.xDistToTurnNav - delta_dist
    self.xDistToTurn = self.xDistToTurnNav
    self.xDistToTurnNext = self.xDistToTurnNext - delta_dist

    if self.xDistToTurnNavLast != self.xDistToTurnNav:
      if self.xDistToTurnNav > self.xDistToTurnMax:
        self.xDistToTurnMax = self.xDistToTurnNav
        self.xDistToTurnMaxCnt = 0
      elif self.xDistToTurnNav < self.xDistToTurnMax:
        self.xDistToTurnMaxCnt += 1
        if self.xDistToTurnMaxCnt > 50: # 5秒
          self.xDistToTurnMax = self.xDistToTurnNav
          self.xDistToTurnMaxCnt = 0
    self.xDistToTurnNavLast = self.xDistToTurnNav
    self.active_count = max(self.active_count - 1, 0)
    self.active_sdi_count = max(self.active_sdi_count - 1, 0)
    self.active_kisa_count = max(self.active_kisa_count - 1, 0)
    if self.active_kisa_count > 0:
      self.active_carrot = 2

    elif self.active_count > 0:
      self.active_carrot = 2 if self.active_sdi_count > 0 else 1
    else:
      self.active_carrot = 1  # 即使无 App 连接，也保持基础控速开启

    if self.autoRoadSpeedLimitOffset >= 0 and self.active_carrot>=1:
      if self.nRoadLimitSpeed >= 30:
        road_speed_limit_offset = self.autoRoadSpeedLimitOffset
        if not self.is_metric:
          road_speed_limit_offset *= CV.KPH_TO_MPH
        limit_speed = self.nRoadLimitSpeed + road_speed_limit_offset
      else:
        limit_speed = 250
    else:
      limit_speed = 250

    if self.active_carrot <= 1:
      self.xSpdType = self.navType = self.xTurnInfo = self.xTurnInfoNext = -1
      self.nSdiType = self.nSdiBlockType = self.nSdiPlusBlockType = -1
      self.nTBTTurnType = self.nTBTTurnTypeNext = -1
      self.roadcate = 8
      self.nGoPosDist = 0
    if self.active_carrot <= 1 or self.active_kisa_count > 0:
      self.update_nav_instruction(sm)

    if self.xSpdType < 0 or (self.xSpdType not in [100,101] and self.xSpdDist <= 0) or (self.xSpdType in [100,101] and self.xSpdDist < -250):
      self.xSpdType = -1
      self.xSpdDist = self.xSpdLimit = 0
    if self.xTurnInfo < 0 or self.xDistToTurn < -50:
      if self.xDistToTurn > 0:
        self.xDistToTurn = 0
      self.xTurnInfo = -1
      self.xDistToTurnNext = 0
      self.xTurnInfoNext = -1

    sdi_speed = 250
    hda_active = False
    ### 测速摄像头, 减速带
    if (self.xSpdDist > 0 or self.xSpdType in [100, 101]) and self.active_carrot > 0:
      safe_sec = self.autoNaviSpeedBumpTime if self.xSpdType == 22 else self.autoNaviSpeedCtrlEnd
      decel = self.autoNaviSpeedDecelRate
      sdi_speed = min(sdi_speed, self.calculate_current_speed(self.xSpdDist, self.xSpdLimit, safe_sec, decel))
      self.active_carrot = 5 if self.xSpdType == 22 else 3
      if self.xSpdType == 4 or (self.xSpdType in [100, 101] and self.xSpdDist <= 0):
        sdi_speed = self.xSpdLimit
        self.active_carrot = 4
    elif CS is not None and CS.speedLimit > 0 and CS.speedLimitDistance > 0:
      sdi_speed = min(sdi_speed,
                      self.calculate_current_speed(CS.speedLimitDistance,
                                                   CS.speedLimit * 3.6 * self.autoNaviSpeedSafetyFactor,
                                                   self.autoNaviSpeedCtrlEnd,
                                                   self.autoNaviSpeedDecelRate))
      hda_active = True

    ### TBT 速度控制
    atc_desired, self.atcType, self.atcSpeed, self.atcDist, atc_speed_up = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfo, self.xDistToTurn, True)
    atc_desired_next, _, _, _, _ = self.update_auto_turn(v_ego*3.6, sm, self.xTurnInfoNext, self.xDistToTurnNext, False)

    if self.autoTurnControl not in [2, 3]:    # 自动转向速度控制
      atc_desired = atc_desired_next = 250

    if self.autoTurnControl not in [1,2]:    # 自动转向控制
      self.atcType = "none"


    speed_n_sources = [
      (atc_desired, "atc"),
      (atc_desired_next, "atc2"),
      (sdi_speed, "hda" if hda_active else "bump" if self.xSpdType == 22 else "section" if self.xSpdType == 4 else "police" if self.xSpdType == 100 else "waze" if self.xSpdType == 101 else "cam"),
      (limit_speed, "road"),
    ]
    if self.turnSpeedControlMode >= 0: # 默认开启，除非显式禁用
      speed_n_sources.append((max(abs(vturn_speed), self.autoCurveSpeedLowerLimit), "vturn"))

    route_speed = max(route_speed * self.mapTurnSpeedFactor, self.autoCurveSpeedLowerLimit)
    if self.turnSpeedControlMode == 2:
      if -500 < self.xDistToTurn < 500:
        speed_n_sources.append((route_speed, "route"))
    elif self.turnSpeedControlMode in [3, 4]:
      speed_n_sources.append((route_speed, "route"))

    # 弯道主动减速策略：对视觉模型的预测弯道速度打 85 折，抑制 AI 盲目自信，迫使提前减速
    model_turn_speed = max(sm['modelV2'].meta.modelTurnSpeed * 0.85, self.autoCurveSpeedLowerLimit)
    if model_turn_speed < 200:
      speed_n_sources.append((model_turn_speed, "model"))

    desired_speed, source = min(speed_n_sources, key=lambda x: x[0])
    original_source = source

    if CS is not None:
      if CS.gasPressed:
        self.gas_override_speed = max(v_ego_kph, self.gas_override_speed)

      if CS.vEgo < 0.1 or desired_speed > 150 or original_source in ["cam", "section", "police"] or CS.brakePressed or road_speed_limit_changed:
        self.gas_override_speed = 0

      if desired_speed < self.gas_override_speed:
        source = "gas"
        desired_speed = self.gas_override_speed

      # 增强调试信息显示到 UI (格式: 来源 | 目标速度 | 道路限速 | 视觉弯道速 | 地图弯道速)
      self.debugText = f" [{source}|{desired_speed:.0f}|L:{limit_speed:.0f}|V:{vturn_speed:.0f}|R:{route_speed:.0f}]"

    left_spd_sec = 100
    left_tbt_sec = 100
    if self.autoNaviCountDownMode > 0:
      if self.xSpdType == 22 and self.autoNaviCountDownMode == 1: # 减速带
        pass
      else:
        if self.xSpdDist > 0:
          left_spd_sec = min(self.left_spd_sec, int(max(self.xSpdDist - v_ego, 1) / max(1, v_ego) + 0.5))

      if self.xDistToTurn > 0:
        left_tbt_sec = int(max(self.xDistToTurn - v_ego, 1) / max(1, v_ego) + 0.5)

    self.left_spd_sec = left_spd_sec
    self.left_tbt_sec = left_tbt_sec
    self.carrot_left_sec = left_tbt_sec

    left_sec = min(left_spd_sec, left_tbt_sec)

    if left_sec > 11:
      self.left_sec = 100
      self.max_left_sec = 100
    else:
      self.sdi_inform = True if source in ["cam", "hda"] else False
      self.max_left_sec = min(11, max(6, int(v_ego_kph/10) + 1))

    if left_sec != self.left_sec:
      if left_sec == self.max_left_sec and self.sdi_inform:
        self.carrot_left_sec = 11
      elif 1 <= left_sec < self.max_left_sec:
        self.carrot_left_sec = left_sec
      elif left_sec == 0 and self.left_sec == 1:
        self.carrot_left_sec = left_sec

      self.left_sec = left_sec


    self._update_cmd()
    msg = messaging.new_message('carrotMan')
    msg.valid = True
    msg.carrotMan.activeCarrot = self.active_carrot
    msg.carrotMan.nRoadLimitSpeed = int(self.nRoadLimitSpeed)
    msg.carrotMan.remote = remote_ip
    msg.carrotMan.xSpdType = int(self.xSpdType)
    msg.carrotMan.xSpdLimit = int(self.xSpdLimit)
    msg.carrotMan.xSpdDist = int(self.xSpdDist)
    msg.carrotMan.xSpdCountDown = int(left_spd_sec)
    msg.carrotMan.xTurnInfo = int(self.xTurnInfo)
    msg.carrotMan.xDistToTurn = int(self.xDistToTurn)
    msg.carrotMan.xTurnCountDown = int(left_tbt_sec)
    msg.carrotMan.atcType = self.atcType
    msg.carrotMan.vTurnSpeed = int(vturn_speed)
    msg.carrotMan.szPosRoadName = self.szPosRoadName + self.debugText
    msg.carrotMan.szTBTMainText = self.szTBTMainText
    msg.carrotMan.desiredSpeed = int(desired_speed)
    msg.carrotMan.desiredSource = source
    msg.carrotMan.carrotCmdIndex = int(self.carrotCmdIndex)
    msg.carrotMan.carrotCmd = self.carrotCmd
    msg.carrotMan.carrotArg = self.carrotArg
    msg.carrotMan.trafficState = self.traffic_state

    msg.carrotMan.xPosSpeed = float(v_ego_kph) 
    msg.carrotMan.xPosAngle = float(self.bearing)
    msg.carrotMan.xPosLat = float(self.vpPosPointLat)
    msg.carrotMan.xPosLon = float(self.vpPosPointLon)

    msg.carrotMan.nGoPosDist = self.nGoPosDist
    msg.carrotMan.nGoPosTime = self.nGoPosTime
    msg.carrotMan.szSdiDescr = self._get_sdi_descr(-1 if self.nSdiType == 0 and self.nSdiDist == 0 else self.nSdiType)

    coords_str = ";".join([f"{x:.2f},{y:.2f},{d:.2f}" for (x, y), d in zip(coords, distances, strict=False)])
    msg.carrotMan.naviPaths = coords_str

    msg.carrotMan.leftSec = int(self.carrot_left_sec)
    msg.carrotMan.roadCate = int(self.roadcate)
    msg.carrotMan.xLeftTurnSec = int(self.carrot_left_sec)
    msg.carrotMan.atcCurvature = float(atc_curvature)
    pm.send('carrotMan', msg)

    inst = messaging.new_message('navInstructionCarrot')
    if self.active_carrot > 1 and self.active_kisa_count <= 0:
      inst.valid = True

      instruction = inst.navInstructionCarrot
      instruction.distanceRemaining = self.nGoPosDist
      instruction.timeRemaining = self.nGoPosTime
      instruction.speedLimit = self.nRoadLimitSpeed / 3.6 if self.nRoadLimitSpeed > 0 else 0
      instruction.maneuverDistance = float(self.nTBTDist)
      instruction.maneuverSecondaryText = self.szNearDirName
      if self.szFarDirName and len(self.szFarDirName):
        instruction.maneuverSecondaryText += "[{}]".format(self.szFarDirName)
      instruction.maneuverPrimaryText = self.szTBTMainText
      instruction.timeRemainingTypical = self.nGoPosTime

      navType, navModifier, xTurnInfo1 = "invalid", "", -1
      if self.nTBTTurnType in nav_type_mapping:
        navType, navModifier, xTurnInfo1 = nav_type_mapping[self.nTBTTurnType]
      navTypeNext, navModifierNext, xTurnInfoNext = "invalid", "", -1
      if self.nTBTTurnTypeNext in nav_type_mapping:
        navTypeNext, navModifierNext, xTurnInfoNext = nav_type_mapping[self.nTBTTurnTypeNext]

      instruction.maneuverType = navType
      instruction.maneuverModifier = navModifier

      maneuvers = []
      if self.nTBTTurnType >= 0:
        maneuver = {}
        maneuver['distance'] = float(self.xDistToTurn)
        maneuver['type'] = navType
        maneuver['modifier'] = navModifier
        maneuvers.append(maneuver)
        if self.nTBTDistNext >= self.nTBTDist:
          maneuver = {}
          maneuver['distance'] = float(self.nTBTDistNext)
          maneuver['type'] = navTypeNext
          maneuver['modifier'] = navModifierNext
          maneuvers.append(maneuver)

      instruction.allManeuvers = maneuvers
    elif sm.alive['navInstruction'] and sm.valid['navInstruction']:
      inst.navInstructionCarrot = sm['navInstruction']

    pm.send('navInstructionCarrot', inst)

  def _update_system_time(self, epoch_time_remote, timezone_remote):
    epoch_time = int(time.time())
    if epoch_time_remote > 0:
      epoch_time_offset = epoch_time_remote - epoch_time
      print(f"epoch_time_offset = {epoch_time_offset}")
      if abs(epoch_time_offset) > 60:
        os.system(f"sudo timedatectl set-timezone {timezone_remote}")
        formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch_time_remote))
        print(f"Setting system time to: {formatted_time}")
        os.system(f'sudo date -s "{formatted_time}"')

  def set_time(self, epoch_time, timezone):
    import datetime
    new_time = datetime.datetime.utcfromtimestamp(epoch_time)
    localtime_path = "/data/etc/localtime"

    no_timezone = False
    try:
      if os.path.getsize(localtime_path) == 0:
        no_timezone = True
    except (FileNotFoundError, OSError):
      no_timezone = True

    diff = datetime.datetime.utcnow() - new_time
    if abs(diff) < datetime.timedelta(seconds=10) and not no_timezone:
      return

    print(f"Setting time to {new_time}, diff={diff}")
    zoneinfo_path = f"/usr/share/zoneinfo/{timezone}"
    if os.path.exists(localtime_path) or os.path.islink(localtime_path):
        try:
            subprocess.run(["sudo", "rm", "-f", localtime_path], check=True)
            print(f"Removed existing file or link: {localtime_path}")
        except subprocess.CalledProcessError as e:
            print(f"Error removing {localtime_path}: {e}")
            return
    try:
        subprocess.run(["sudo", "ln", "-s", zoneinfo_path, localtime_path], check=True)
        print(f"Timezone successfully set to: {timezone}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to set timezone to {timezone}: {e}")


    try:
      subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
    except subprocess.CalledProcessError:
      print("timed.failed_setting_time")

  def update(self, json):
    if json is None:
      return
    if "carrotIndex" in json:
      self.carrotIndex = int(json.get("carrotIndex"))

    if self.carrotIndex % 60 == 0 and "epochTime" in json:
      # op 使用 ntp，此处作为备用同步
      timezone_remote = json.get("timezone", "Asia/Seoul")

      if not PC:
        self.set_time(int(json.get("epochTime")), timezone_remote)

    if "carrotCmd" in json:
      self.carrotCmdIndex = self.carrotIndex
      self.carrotCmd = json.get("carrotCmd")
      self.carrotArg = json.get("carrotArg")
      print(f"carrotCmd = {self.carrotCmd}, {self.carrotArg}")

    self.active_count = 80
    now = time.monotonic()

    if "goalPosX" in json:
      self.goalPosX = float(json.get("goalPosX", self.goalPosX))
      self.goalPosY = float(json.get("goalPosY", self.goalPosY))
      self.szGoalName = json.get("szGoalName", self.szGoalName)

    if "nRoadLimitSpeed" in json:
      self.active_sdi_count = self.active_sdi_count_max
      ### roadLimitSpeed
      nRoadLimitSpeed = int(json.get("nRoadLimitSpeed", 20))
      if nRoadLimitSpeed > 0:
        if nRoadLimitSpeed > 200:
          nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
        elif nRoadLimitSpeed == 120:
          nRoadLimitSpeed = 115 # 120 -> 115 fix bug
      else:
        nRoadLimitSpeed = 30
      if self.nRoadLimitSpeed != nRoadLimitSpeed:
        self.nRoadLimitSpeed_counter += 1
        if self.nRoadLimitSpeed_counter > 5:
          self.nRoadLimitSpeed = nRoadLimitSpeed
      else:
        self.nRoadLimitSpeed_counter = 0

      ### SDI
      self.nSdiType = int(json.get("nSdiType", -1))
      self.nSdiSpeedLimit = int(json.get("nSdiSpeedLimit", 0))
      self.nSdiSection = int(json.get("nSdiSection", -1))
      self.nSdiDist = int(json.get("nSdiDist", -1))
      self.nSdiBlockType = int(json.get("nSdiBlockType", -1))
      self.nSdiBlockSpeed = int(json.get("nSdiBlockSpeed", 0))
      self.nSdiBlockDist = int(json.get("nSdiBlockDist", 0))

      self.nSdiPlusType = int(json.get("nSdiPlusType", -1))
      self.nSdiPlusSpeedLimit = int(json.get("nSdiPlusSpeedLimit", 0))
      self.nSdiPlusDist = int(json.get("nSdiPlusDist", 0))
      self.nSdiPlusBlockType = int(json.get("nSdiPlusBlockType", -1))
      self.nSdiPlusBlockSpeed = int(json.get("nSdiPlusBlockSpeed", 0))
      self.nSdiPlusBlockDist = int(json.get("nSdiPlusBlockDist", 0))
      self.roadcate = int(json.get("roadcate", 0))

      ## GuidePoint
      self.nTBTDist = int(json.get("nTBTDist", 0))
      self.nTBTTurnType = int(json.get("nTBTTurnType", -1))
      self.szTBTMainText = json.get("szTBTMainText", "")
      self.szNearDirName = json.get("szNearDirName", "")
      self.szFarDirName = json.get("szFarDirName", "")

      self.nTBTNextRoadWidth = int(json.get("nTBTNextRoadWidth", 0))
      self.nTBTDistNext = int(json.get("nTBTDistNext", 0))
      self.nTBTTurnTypeNext = int(json.get("nTBTTurnTypeNext", -1))
      self.szTBTMainTextNext = json.get("szTBTMainText", "")

      self.nGoPosDist = int(json.get("nGoPosDist", 0))
      self.nGoPosTime = int(json.get("nGoPosTime", 0))
      self.szPosRoadName = json.get("szPosRoadName", "")
      if self.szPosRoadName == "null":
        self.szPosRoadName = ""

      self.vpPosPointLatNavi = float(json.get("vpPosPointLat", 0.0))
      self.vpPosPointLonNavi = float(json.get("vpPosPointLon", 0.0))
      if self.vpPosPointLatNavi != 0.0:
        self.last_update_gps_time_navi = self.last_calculate_gps_time = now
        self.nPosAngle = float(json.get("nPosAngle", self.nPosAngle))

      self.nPosSpeed = float(json.get("nPosSpeed", self.nPosSpeed))
      self._update_tbt()
      self._update_sdi()
      print(
        f"sdi = {self.nSdiType}, {self.nSdiSpeedLimit}, {self.nSdiPlusType}, " +
        f"tbt = {self.nTBTTurnType}, {self.nTBTDist}, " +
        f"next = {self.nTBTTurnTypeNext}, {self.nTBTDistNext}"
      )

    # 如果 3 秒内没有导航数据，则通过手机 GPS 更新
    if "latitude" in json:
      self.nPosAnglePhone = float(json.get("heading", self.nPosAngle))
      self.phone_latitude = float(json.get("latitude", self.vpPosPointLatNavi))
      self.phone_longitude = float(json.get("longitude", self.vpPosPointLonNavi))
      self.phone_gps_accuracy = float(json.get("accuracy", 0))
      if self.phone_gps_accuracy < 15.0:
        self.phone_gps_frame += 1
      if (now - self.last_update_gps_time_navi) > 3.0:
        self.vpPosPointLatNavi = self.phone_latitude
        self.vpPosPointLonNavi = self.phone_longitude

        self.nPosAngle = self.nPosAnglePhone
        self.last_update_gps_time_phone = self.last_calculate_gps_time = now        
        self.nPosSpeed = float(json.get("gps_speed", 0))
        print(f"手机 GPS: {self.vpPosPointLatNavi}, {self.vpPosPointLonNavi}, 精度: {self.phone_gps_accuracy}, 速度: {self.nPosSpeed}")


import traceback

def main():
  print("CarrotManager Started")
  from openpilot.selfdrive.carrot.carrot_man import CarrotMan
  carrot_man = CarrotMan()

  print(f"CarrotMan {carrot_man}")
  threading.Thread(target=carrot_man.kisa_app_thread).start()
  while True:
    try:
      carrot_man.carrot_man_thread()
    except Exception as e:
      print(f"carrot_man 错误...: {e}")
      traceback.print_exc()
      time.sleep(10)


if __name__ == "__main__":
  main()
