#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import time
import traceback

from cereal import car, log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version
from openpilot.system.hardware.hw import Paths

def get_default_params():
  default_params : list[tuple[str, str | bytes]] = [
    # kans
    ("LongPitch", "0"),  # 纵向俯仰补偿 (恢复备份值 0)
    ("EVTable", "0"),  # 电动车加速响应表 (恢复备份值 0)
    ("CompletedTrainingVersion", "0"),  # 已完成训练版本
    ("DisengageOnAccelerator", "0"),  # 踩油门时是否退出巡航
    ("GsmMetered", "1"),  # GSM 流量计费开启
    ("HasAcceptedTerms", "0"),  # 已接受用户协议
    ("LanguageSetting", "main_zh-CHS"),  # 语言设置 (简体中文)
    ("OpenpilotEnabledToggle", "1"),  # 启用 openpilot
    ("LongitudinalPersonality", "0"),  # 纵向个性化 (恢复备份值 0)
    ("IsMetric", "1"),  # 使用公制单位 (km/h)
    ("RecordAudio", "1"),  # 开启录音

    ("SearchInput", "0"),  # 搜索输入设置
    ("GMapKey", "0"),  # Google 地图 API 密钥
    ("MapboxStyle", "0"),  # Mapbox 样式设置

    ("LongitudinalPersonalityMax", "4"),  # 纵向个性化最大档位 (恢复备份值 4)
    ("ShowDebugUI", "0"),  # 调试信息: 0:关闭, 1:开启, 2:更多调试项
    ("ShowTpms", "1"),  # 胎压信息: 0:不显示, 1:显示 (0~3 档位)
    ("ShowDateTime", "1"),  # 时间信息: 0:无, 1:时间/日期, 2:仅时间, 3:仅日期
    ("ShowPathEnd", "1"),  # 路径终点指示线: 0:不显示, 1:显示
    ("ShowCustomBrightness", "100"),  # 自定义屏幕亮度 (百分比)
    ("ShowLaneInfo", "2"),  # 车道线信息显示等级: 0:无, 1:基本, 2:详细
    ("ShowRadarInfo", "1"),  # 雷达目标信息: 0:不显示, 1:显示
    ("ShowDeviceState", "1"),  # 设备状态(CPU/温度): 0:不显示, 1:显示
    ("ShowRouteInfo", "1"),  # 路由信息: 0:不显示, 1:显示
    ("ShowPathMode", "9"),  # 路径画法(无车道/Laneless): 0:普通, 1,2:矩形, 3,4:^^, 5,6:矩形, 7,8:^^, 9~11:平滑^^, 12:平滑2^^, 13:3条线, 14:2条线, 15:1条线
    ("ShowPathColor", "13"),  # 巡航开启路径颜色: (+10:描边)0:红,1:橙,2:黄,3:绿,4:蓝,5:靛蓝,6:紫罗兰,7:棕,8:白,9:黑
    ("ShowPathColorCruiseOff", "19"),  # 巡航关闭路径颜色: 同上 (+10:描边)
    ("ShowPathModeLane", "14"),  # 路径画法(车道模式/LaneMode): 取值同 ShowPathMode
    ("ShowPathColorLane", "13"),  # 车道模式路径颜色: 取值同 ShowPathColor
    ("ShowPlotMode", "7"),  # 默认显示转向角波形图 (Y:Actual, G:Target, O:Offset)
    ("AutoCruiseControl", "0"),  # 自动巡航(仅HKG): 0:关闭, 1:软保持+自动巡航, 2:软保持异常车辆用
    ("CruiseEcoControl", "0"),  # 巡航节能: 0:关闭, 1~10:临时提高设定速度值(km/h)以省油
    ("CarrotCruiseDecel", "-1"),  # Carrot 模式减速量 (m/s², -1:自动)
    ("CarrotCruiseAtcDecel", "-1"),  # Carrot ATC 减速量 (m/s², -1:自动)
    ("CommaLongAcc", "0"),  # Comma 纵向加速度控制: 0:关闭, 1:开启
    ("AutoGasTokSpeed", "0"),  # 自动油门(加速)启用速度 (km/h, 0:不使用)
    ("AutoGasSyncSpeed", "1"),  # 自动同步当前车速到巡航设定: 0:关闭, 1:开启
    ("AutoEngage", "0"),  # 启动自动接管: 0:关闭, 1:仅自动启用转向, 2:转向+巡航待机(可自动开启巡航)
    ("DisableMinSteerSpeed", "0"),  # 禁用最低转向速度限制 (允许 0 速转向)
    ("SoftHoldMode", "0"),  # 自动驻车保持模式

    ("AutoSpeedUptoRoadSpeedLimit", "100"),  # 自动调节至路段限速 (百分比, 100=完全按限速)
    ("AutoRoadSpeedAdjust", "100"),  # 公路速度自动调节比例 (百分比)
    ("AutoCurveSpeedLowerLimit", "30"),  # 弯道降速下限速度 (km/h)
    ("AutoCurveSpeedFactor", "120"),  # 弯道降速灵敏度系数 (百分比, >100 更早降速)
    ("AutoCurveSpeedAggressiveness", "100"),  # 弯道降速激进程度 (百分比, >100 减速更强)

    ("AutoTurnControl", "2"),  # 自动转向控制(ATC): 0:关闭, 1:仅导航变道, 2:导航变道+转弯速度控制, 3:仅转弯速度控制
    ("AutoTurnControlSpeedTurn", "20"),  # 自动转向: 转弯目标速度 (km/h, 0:不使用)
    ("AutoTurnControlTurnEnd", "6"),  # 自动转向: 转弯结束提前量 (距离=速度x时间, 秒)
    ("AutoTurnMapChange", "0"),  # 自动转向: 转弯时自动切换地图渲染: 0:关闭, 1:开启

    ("NightGainEnable", "1"),  # 夜间增益增强: 0:关闭, 1:开启(画面过暗时自动增亮长焦,约5秒后生效)
    ("NightGainTargetY", "90"),  # 夜间增益目标亮度 (20~200, 画面增亮到的平均亮度值)
    ("NightGainMax", "20"),  # 夜间增益最大倍数 x10 (10~40, 即 1.0~4.0 倍)
    ("NightWideSwitch", "1"),  # 夜间广角兜底: 0:关闭, 1:开启(长焦极端过暗时广角接管模型主输入,约5秒后生效)
    ("NightWideDarkY", "48"),  # 广角接管阈值: 长焦平均亮度低于此值(10~120)连续20帧切广角
    ("NightWideBrightY", "60"),  # 恢复长焦阈值: 长焦平均亮度高于此值(20~160)连续20帧切回

    ("AutoNaviSpeedCtrlEnd", "7"),  # 导航限速控制结束提前量
    ("AutoNaviSpeedCtrlMode", "3"),  # 导航限速模式: 0:不减速, 1:仅测速摄像头, 2:+减速带, 3:+移动测速
    ("AutoNaviSpeedBumpTime", "1"),  # 减速带提前减速时间 (秒)
    ("AutoNaviSpeedBumpSpeed", "35"),  # 减速带目标通过速度 (km/h)
    ("AutoNaviSpeedSafetyFactor", "105"),  # 测速摄像头安全裕度 (百分比, 105=限速x1.05)
    ("AutoNaviSpeedDecelRate", "120"),  # 导航限速减速率 (x0.01 m/s², 值越小越早开始减速)
    ("AutoRoadSpeedLimitOffset", "0"),  # 路段限速偏置 (km/h, -1:不使用路段限速)
    ("AutoNaviCountDownMode", "2"),  # 导航倒计时: 0:关闭, 1:导航+摄像头, 2:导航+摄像头+减速带
    ("TurnSpeedControlMode", "2"),  # 转向速度控制: 0:关闭, 1:仅视觉, 2:视觉+导航, 3:仅导航
    ("CarrotSmartSpeedControl", "0"),  # 智能速度控制: 0:关闭, 1:加速智能, 2:减速智能, 3:全部
    ("MapTurnSpeedFactor", "90"),  # 地图转弯速度补偿系数 (百分比, 基于导航数据的过弯限速)
    ("ModelTurnSpeedFactor", "0"),  # 模型转弯速度补偿系数 (x0.1, 基于视觉模型曲率的过弯限速, 0:不使用)
    ("StoppingAccel", "-50"),  # 停止时的加速度限制 (恢复备份值 -50)
    ("StopDistanceCarrot", "450"),  # Carrot 停止距离设置 (恢复备份值 450)
    ("JLeadFactor3", "0"),  # 跟车距离补偿系数 (前车相关) (恢复备份值 0)
    ("CruiseButtonMode", "0"),  # 巡航按钮模式: 0:普通, 1:用户1, 2:用户2
    ("CancelButtonMode", "0"),  # 取消按钮模式: 0:仅取消纵向, 1:取消纵向+横向
    ("LfaButtonMode", "0"),  # LFA按钮模式: 0:普通, 1:减速&停止&前车就绪
    ("CruiseButtonTest1", "8"),  # 巡航按钮测试参数 1
    ("CruiseButtonTest2", "30"),  # 巡航按钮测试参数 2
    ("CruiseButtonTest3", "1"),  # 巡航按钮测试参数 3
    ("CruiseSpeedUnit", "1"),  # 巡航速度增量单位 (恢复备份值 1)
    ("CruiseSpeedUnitBasic", "5"),  # 巡航步进基础单位 (恢复备份值 5)
    ("CruiseSpeed1", "30"),  # 巡航记忆速度 1
    ("CruiseSpeed2", "50"),  # 巡航记忆速度 2
    ("CruiseSpeed3", "80"),  # 巡航记忆速度 3
    ("CruiseSpeed4", "110"),  # 巡航记忆速度 4
    ("CruiseSpeed5", "130"),  # 巡航记忆速度 5
    ("PaddleMode", "0"),  # 方向盘拨片操作模式: 0:关闭, 1:开启
    ("MyDrivingMode", "1"),  # 驾驶模式: 1:舒适, 2:经济, 3:标准, 4:运动
    ("MyDrivingModeAuto", "0"),  # 自动切换驾驶模式: 0:关闭, 1:开启
    ("TrafficLightDetectMode", "2"),  # 红绿灯检测: 0:不检测, 1:仅红灯停, 2:红灯停+绿灯起步
    ("CruiseMaxVals0", "140"),  # 巡航最大设定值 - 段 0 (恢复备份值 140)
    ("CruiseMaxVals1", "160"),  # 巡航最大设定值 - 段 1 (恢复备份值 160)
    ("CruiseMaxVals2", "140"),  # 巡航最大设定值 - 段 2 (恢复备份值 140)
    ("CruiseMaxVals3", "100"),  # 巡航最大设定值 - 段 3 (恢复备份值 100)
    ("CruiseMaxVals4", "60"),  # 巡航最大设定值 - 段 4 (恢复备份值 60)
    ("CruiseMaxVals5", "50"),  # 巡航最大设定值 - 段 5 (恢复备份值 50)
    ("CruiseMaxVals6", "50"),  # 巡航最大设定值 - 段 6 (恢复备份值 50)
    ("LongTuningKpV", "100"),  # 纵向调优 P 增益 (百分比)
    ("LongTuningKiV", "0"),  # 纵向调优 I 增益 (百分比)
    ("LongTuningKf", "100"),  # 纵向调优 F 增益 (百分比)
    ("LongActuatorDelay", "20"),  # 纵向执行机构延迟补偿
    ("VEgoStopping", "50"),  # 车速判定为停止的阈值
    ("RadarReactionFactor", "100"),  # 雷达响应敏感度系数 (百分比, >100 更快响应)
    ("EnableRadarTracks", "0"),  # 雷达轨迹模式: -1:始终用原厂SCC雷达, 0:SCC雷达仅低速兜底(默认,纯视觉优先), 1:雷达轨迹+视觉融合, 2:雷达轨迹+SCC始终兜底, 3:雷达轨迹+加塞/隐身车检测(需RadarLatFactor>0配合)
    ("RadarLatFactor", "0"),  # 雷达侧向位置修正系数 (x0.01, >0 启用加塞Cut-in检测)
    ("EnableEscc", "0"),  # 增强型SCC(ESCC): 0:关闭(探测到0x2AB报文时自动启用), 1:手动强制启用(原车ESCC报文解析+纵向控制反馈)
    ("EnableCornerRadar", "2"),  # 原车角雷达(BSD)加塞检测: 0:关闭, >0:启用——读取原车CAN角雷达报文(ADRV_0x162/0x1ea)探测侧方2.5m内加塞车辆并注入前车目标, 无需加装硬件
    ("HyundaiCameraSCC", "0"),  # 现代: 0:关闭, 1:SCC的CAN线接到CAM(雷达盒方案), 2:同步巡航状态, 3:原厂纵向控制
    ("IsLdwsCar", "0"),  # 是否为无 LKAS 按钮车辆: 0:否, 1:是(LDW车辆用LKAS按钮替代)
    ("CanfdHDA2", "0"),  # CAN-FD: 0:关闭(HDA1), 1:HDA2协议, 2:HDA2+BSM盲区监测报文
    ("CanfdDebug", "0"),  # CAN-FD 数据调试日志: 0:关闭, 1:开启
    ("SoundVolumeAdjust", "100"),  # 全局系统音量大小百分比
    ("SoundVolumeAdjustEngage", "10"),  # 开启巡航时的音量淡入比例
    ("TFollowGap1", "110"),  # 一档跟车距离设置 (秒 x 100)
    ("TFollowGap2", "120"),  # 二档跟车距离设置 (秒 x 100)
    ("TFollowGap3", "140"),  # 三档跟车距离设置 (秒 x 100)
    ("TFollowGap4", "160"),  # 四档跟车距离设置 (秒 x 100)
    ("DynamicTFollow", "0"),  # 动态跟车距离: 0:固定档位, 1:随车速动态调整
    ("AChangeCostStarting", "10"),  # 起步时的加速度变化代价 (平顺度限制)
    ("TrafficStopDistanceAdjust", "-150"),  # 红绿灯前停止距离偏置 (恢复备份值 -150)
    ("ComfortBrake", "240"),  # 舒适制动参数 (x0.01)
    ("DynamicTFollowLC", "100"),  # 变道时的动态跟车系数
    ("HapticFeedbackWhenSpeedCamera", "0"),  # 测速相机报警时开启震动反馈
    ("UseLaneLineSpeed", "0"),  # 车道线模式速度: 0:始终Laneless, >0:低于该速度(km/h)进入车道线模式(有车道线时)
    ("PathOffset", "0"),  # 路径侧向偏移调整 (cm, 0:居中)
    ("UseLaneLineCurveSpeed", "0"),  # 车道线模式弯道限速: 0:关闭, >0:高于该速度(km/h)才启用
    ("AdjustLaneOffset", "0"),  # 车道偏移调整 (cm, 0:居中)
    ("AdjustCurveOffset", "0"),  # 弯道偏移修正 (cm, 0:保持居中)
    ("LaneChangeNeedTorque", "0"),  # 换道触发方式: -1:禁用变道, 0:转向灯即变道(无需扭矩), 1:需方向盘扭矩介入
    ("LaneChangeDelay", "0"),  # 自动换道前的延迟确认时间 (x0.1 秒)
    ("LaneChangeBsd", "0"),  # 变道盲区策略: 0:BSD报警时仍允许变道(宽松), 1:BSD报警禁止变道(安全), -1:不使用BSD
    ("MaxAngleFrames", "89"),  # 大转角保持最大允许帧数
    ("LateralTorqueCustom", "0"),  # 启用自定义扭矩控制参数 (恢复备份值 0)
    ("LateralTorqueAccelFactor", "2500"),  # 侧向扭矩加速度增益系数 (恢复备份值 2500)
    ("LateralTorqueFriction", "100"),  # 侧向扭矩摩擦力静态补偿 (恢复备份值 100)
    ("LateralTorqueKpV", "100"),  # 侧向扭矩 P 比例增益 (恢复备份值 100)
    ("LateralTorqueKiV", "10"),  # 侧向扭矩 I 积分增益 (恢复备份值 10)
    ("LateralTorqueKf", "100"),  # 侧向扭矩 FF 前馈增益 (恢复备份值 100)
    ("LateralTorqueKd", "0"),  # 侧向扭矩 D 微分增益
    ("LatMpcPathCost", "200"),  # 侧向 MPC 控制路径代价
    ("LatMpcMotionCost", "7"),  # 侧向 MPC 控制运动代价
    ("LatMpcAccelCost", "120"),  # 侧向 MPC 加速度代价系数
    ("LatMpcJerkCost", "4"),  # 侧向 MPC Jerk 代价系数 (恢复备份值 4)
    ("LatMpcSteeringRateCost", "800"),  # 侧向 MPC 转向速率代价系数 (调低转向更积极; 800=防摇晃平衡值, 备份中的 7 为极激进)
    ("LateralTorqueFilter", "0"),  # 横向扭矩低通滤波 (默认关闭)
    ("LatMpcInputOffset", "4"),  # 侧向 MPC 路径输入偏置
    ("CustomSteerMax", "0"),  # 自定义最大转向扭矩上限 (0:用车厂默认, >0:自定义Nm值)
    ("CustomSteerDeltaUp", "0"),  # 自定义最大增扭速率 (0:用车厂默认, >0:自定义值)
    ("CustomSteerDeltaDown", "0"),  # 自定义最大减扭速率 (0:用车厂默认, >0:自定义值)
    ("CustomSteerDeltaUpLC", "0"),  # 变道时最大增扭速率 (0:用车厂默认, >0:自定义值)
    ("CustomSteerDeltaDownLC", "0"),  # 变道时最大减扭速率 (0:用车厂默认, >0:自定义值)
    ("SpeedFromPCM", "2"),  # 巡航速度来源: 1:车辆仪表PCM, 2:openpilot内部管理 (丰田必须1, 本田设3)
    ("SteerActuatorDelay", "0"),  # 转向执行延迟补偿时间 (恢复备份值 0)
    ("LatSmoothSec", "13"),  # 横向平滑处理时长系数 (恢复备份值 13)
    ("MaxTimeOffroadMin", "60"),  # 无操作后最大休眠前运行时长 (分)
    ("DisableDM", "1"),  # 驾驶员监控: 0:开启DM, 1:禁用DM（慎用）
    ("EnableConnect", "0"),  # 联网连接: 0:禁用(设备可能被Comma封禁), 1/2:启用
    ("MuteDoor", "0"),  # 屏蔽车门未关报警: 0:不屏蔽, 1:屏蔽
    ("MuteSeatbelt", "0"),  # 屏蔽安全带未系报警: 0:不屏蔽, 1:屏蔽
    ("RecordRoadCam", "0"),  # 循环记录前方路面视频: 0:不记录, 1:前摄像头, 2:前摄像头+广角摄像头
    ("HDPuse", "0"),  # HDP(CCNC): 0:关闭, 1:使用APN导航时启用, 2:始终使用
    ("CruiseOnDist", "400"),  # 距离检测开启巡航距离
    ("AutoPandaUpdate", "0"),  # 默认开启 Panda 固件自动更新
    ("HotspotOnBoot", "0"),  # 开机自动开启 WiFi 热点
    ("SoftwareMenu", "1"),  # 开启软件菜单显示
    ("CustomSR", "0"),  # 自定义转向比: 0:用车厂默认SR, >0:自定义值
    ("SteerRatioRate", "100"),  # 动态转向比调节速率 (百分比)
    ("NNFF", "1"),  # 神经网络前馈横向控制(NNFF): 0:关闭(传统控制), 1:开启(需重启)
    ("NNFFLite", "0"),  # 轻量级NNFF: 0:关闭, 1:开启(需重启)
    ("ShareData", "0"),  # 分享匿名驾驶数据以持续改进

    # 断开连接自动关机功能
    ("EnableDisconnectShutdown", "0"),  # 默认关闭，检测到断开连接后自动关机
    ("DisconnectShutdownDelay", "5"),  # 断开连接后等待关机的延迟秒数
    ("ShowDebugLog", "0"),  # 显示调试日志等级 (0: 关闭, 128: 开启全部)
  ]
  return default_params

def set_default_params():
  params = Params()
  default_params = get_default_params()
  # safe remove to prevent ValueError if keys don't match exactly
  for key_to_remove in (("GMapKey", "0"), ("CompletedTrainingVersion", "0"), ("LanguageSetting", "main_en"), ("GsmMetered", "1")):
    if key_to_remove in default_params:
      default_params.remove(key_to_remove)
  for k, v in default_params:
    params.put(k, v)
    print(f"SetToDefault[{k}]={v}")

def get_default_params_key():
  default_params = get_default_params()
  all_keys = [key for key, _ in default_params]
  return all_keys

def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  default_params = get_default_params()

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # 自动迁移强覆盖逻辑已停用，遵循 params_backup_20260831.json 的横向控制配置
  # if not params.get_bool("SantaFeParamsTuned3"):
  #   params.put("CustomSR", "158")
  #   params.put("LateralTorqueAccelFactor", "2900")
  #   params.put("LateralTorqueFriction", "180")
  #   params.put("LateralTorqueKpV", "50")
  #   params.put("LateralTorqueKiV", "5")
  #   params.put("LateralTorqueKd", "0")
  #   params.put("SteerActuatorDelay", "15")
  #   params.put_bool("SantaFeParamsTuned3", True)



  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)
  params.put("HardwareSerial", serial)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs (graceful shutdown)
  for p in managed_processes.values():
    p.stop(block=False)

  # 极度优化：给予最多 1.5 秒的时间让 loggerd, camerad 等进程优雅结束 (写入视频 MOOV atom 等)
  t = time.monotonic()
  while time.monotonic() - t < 1.5:
    if all(p.proc is None or p.proc.exitcode is not None for p in managed_processes.values()):
      break
    time.sleep(0.1)

  # 确保所有未落盘的数据 (如数据库、视频) 被刷入磁盘，防止文件损坏
  os.system("sync")

  # 强制杀死仍未退出的顽固进程，防止死锁导致重启卡死
  for p in managed_processes.values():
    if p.proc is not None and p.proc.exitcode is None:
      p.signal(signal.SIGKILL)

  cloudlog.info("everything is dead or synced")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  if params.get("HardwareC3xLite"):
    ignore += ["micd", "soundd", "loggerd"]
    params.put("RecordAudio", "0")

  sm = messaging.SubMaster(['deviceState', 'carParams', 'carState'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  print_timer = 0

  started_prev = False
  gear_prev = car.CarState.GearShifter.unknown
  ever_driven = False
  driving_start_time = 0.0
  parking_start_time = 0.0

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    # === 优化：带 1s 延时过滤的换挡+手刹自动关机逻辑 ===
    if sm.updated['carState']:
      car_state = sm['carState']
      gear = car_state.gearShifter
      parking_brake = car_state.parkingBrake
      now = time.monotonic()

      # 1. 行驶确认：在 D 或 R 档持续停留超过 1s，才标记为已驱动
      if gear in (car.CarState.GearShifter.drive, car.CarState.GearShifter.reverse):
        if driving_start_time == 0:
          driving_start_time = now
        elif now - driving_start_time >= 1.0:
          ever_driven = True
        parking_start_time = 0 # 驱动时重置停车计时
      else:
        driving_start_time = 0

      # 2. 关机确认：曾驱动过 + 处于 N/P 档 + 手刹拉起 -> 持续 1s 则关机
      is_parking_gear = gear in (car.CarState.GearShifter.neutral, car.CarState.GearShifter.park)
      if ever_driven and is_parking_gear and parking_brake:
        if parking_start_time == 0:
          parking_start_time = now
        elif now - parking_start_time >= 1.0:
          cloudlog.warning(f"Auto-Shutdown: Confirmed parking state for 1s. Powering off...")
          params.put_bool("DoShutdown", True)
      else:
        parking_start_time = 0

      gear_prev = gear

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    if running != getattr(manager_thread, 'last_running', ''):
      print(running)
      manager_thread.last_running = running
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break

def main() -> None:
  manager_init()

  # 获取 openpilot 根目录
  OPENPILOT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  supported_path = Params().get_param_path()

  brands = ["hyundai", "gm", "toyota", "mazda", "tesla", "honda", "volkswagen"]
  for brand in brands:
    values_py = os.path.join(OPENPILOT_ROOT, "opendbc", "car", brand, "values.py")
    output_file = os.path.join(supported_path, f"SupportedCars_{brand}" if brand != "hyundai" else "SupportedCars")
    # 优化：如果文件已存在则跳过生成，避免每次启动都执行 7 个 python 进程
    if os.path.isfile(values_py) and not os.path.exists(output_file):
      os.system(f"python {values_py} > {output_file}")
    elif not os.path.isfile(values_py):
      print(f"WARNING: {values_py} not found, skipping.")

  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    manager_cleanup()
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    manager_cleanup()
    HARDWARE.shutdown()

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    params = Params()
    manager_cleanup()
    if params.get_bool("DoReboot"):
      cloudlog.warning("reboot")
      HARDWARE.reboot()
    elif params.get_bool("DoShutdown"):
      cloudlog.warning("shutdown")
      HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
