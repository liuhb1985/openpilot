#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import traceback

from cereal import log
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
    ("LongPitch", "1"),  # 纵向俯仰补偿
    ("EVTable", "1"),  # 电动车加速响应表
    ("CompletedTrainingVersion", "0"),  # 已完成训练版本
    ("DisengageOnAccelerator", "0"),  # 踩油门时是否退出巡航
    ("GsmMetered", "1"),  # GSM 流量计费开启
    ("HasAcceptedTerms", "0"),  # 已接受用户协议
    ("LanguageSetting", "main_zh-CHS"),  # 语言设置 (简体中文)
    ("OpenpilotEnabledToggle", "1"),  # 启用 openpilot
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),  # 纵向个性化 (跟车距离模式)
    ("IsMetric", "1"),  # 使用公制单位 (km/h)
    ("RecordAudio", "1"),  # 开启录音

    ("SearchInput", "0"),  # 搜索输入设置
    ("GMapKey", "0"),  # Google 地图 API 密钥
    ("MapboxStyle", "0"),  # Mapbox 样式设置

    ("LongitudinalPersonalityMax", "3"),  # 纵向个性化最大档位
    ("ShowDebugUI", "0"),  # 显示 UI 调试信息
    ("ShowTpms", "1"),  # 显示胎压监测
    ("ShowDateTime", "1"),  # 显示当前日期和时间
    ("ShowPathEnd", "1"),  # 显示路径末端指示线
    ("ShowCustomBrightness", "100"),  # 自定义屏幕亮度
    ("ShowLaneInfo", "2"),  # 显示车道线信息
    ("ShowRadarInfo", "1"),  # 显示雷达目标信息
    ("ShowDeviceState", "1"),  # 显示设备状态 (CPU/温度)
    ("ShowRouteInfo", "1"),  # 显示路由信息
    ("ShowPathMode", "0"),  # 路径显示模式
    ("ShowPathColor", "13"),  # 巡航开启时的路径颜色
    ("ShowPathColorCruiseOff", "19"),  # 巡航关闭时的路径颜色
    ("ShowPathModeLane", "0"),  # 车道线路径模式
    ("ShowPathColorLane", "13"),  # 车道线路径颜色
    ("ShowPlotMode", "0"),  # 显示实时波形图
    ("AutoCruiseControl", "1"),  # 自动巡航控制开关
    ("CruiseEcoControl", "0"),  # 巡航节能模式
    ("CarrotCruiseDecel", "-1"),  # Carrot 模式减速量
    ("CarrotCruiseAtcDecel", "-1"),  # Carrot ATC 减速量
    ("CommaLongAcc", "0"),  # 使用 Comma 纵向加速度控制
    ("AutoGasTokSpeed", "0"),  # 自动加油启动速度
    ("AutoGasSyncSpeed", "1"),  # 自动加油同步速度
    ("AutoEngage", "0"),  # 自动启用功能
    ("DisableMinSteerSpeed", "0"),  # 禁用最低转向速度限制 (允许 0 速转向)
    ("SoftHoldMode", "0"),  # 自动驻车保持模式

    ("AutoSpeedUptoRoadSpeedLimit", "0"),  # 自动调节至路段限速
    ("AutoRoadSpeedAdjust", "50"),  # 公路速度自动调节偏置
    ("AutoCurveSpeedLowerLimit", "30"),  # 弯道降速下限速度
    ("AutoCurveSpeedFactor", "120"),  # 弯道降速灵敏度系数
    ("AutoCurveSpeedAggressiveness", "100"),  # 弯道降速激进程度

    ("AutoTurnControl", "2"),  # 自动转弯控制
    ("AutoTurnControlSpeedTurn", "20"),  # 转弯目标速度
    ("AutoTurnControlTurnEnd", "6"),  # 转弯提前结束距离
    ("AutoTurnMapChange", "0"),  # 转弯时地图渲染切换

    ("AutoNaviSpeedCtrlEnd", "7"),  # 导航限速控制结束提前量
    ("AutoNaviSpeedCtrlMode", "2"),  # 导航限速控制模式
    ("AutoNaviSpeedBumpTime", "1"),  # 减速带提前减速时间
    ("AutoNaviSpeedBumpSpeed", "35"),  # 减速带目标通过速度
    ("AutoNaviSpeedSafetyFactor", "105"),  # 导航限速安全偏置系数
    ("AutoNaviSpeedDecelRate", "120"),  # 导航限速减速率
    ("AutoRoadSpeedLimitOffset", "-1"),  # 路段限速偏置
    ("AutoNaviCountDownMode", "2"),  # 导航限速倒计时模式
    ("TurnSpeedControlMode", "1"),  # 转向速度控制模式 (0: 关闭, 1: 开启)
    ("CarrotSmartSpeedControl", "0"),  # Carrot 智能速度控制
    ("MapTurnSpeedFactor", "90"),  # 地图转弯速度补偿系数
    ("ModelTurnSpeedFactor", "0"),  # 模型转弯速度补偿系数
    ("StoppingAccel", "0"),  # 停止时的加速度限制
    ("StopDistanceCarrot", "550"),  # Carrot 停止距离设置
    ("JLeadFactor3", "100"),  # 跟车距离补偿系数 (前车相关)
    ("CruiseButtonMode", "0"),  # 巡航按钮操作模式
    ("CancelButtonMode", "0"),  # 取消按钮操作模式
    ("LfaButtonMode", "0"),  # LFA 按钮操作模式
    ("CruiseButtonTest1", "8"),  # 巡航按钮测试参数 1
    ("CruiseButtonTest2", "30"),  # 巡航按钮测试参数 2
    ("CruiseButtonTest3", "1"),  # 巡航按钮测试参数 3
    ("CruiseSpeedUnit", "10"),  # 巡航速度增量单位
    ("CruiseSpeedUnitBasic", "5"),  # 巡航步进基础单位
    ("CruiseSpeed1", "30"),  # 巡航记忆速度 1
    ("CruiseSpeed2", "50"),  # 巡航记忆速度 2
    ("CruiseSpeed3", "80"),  # 巡航记忆速度 3
    ("CruiseSpeed4", "110"),  # 巡航记忆速度 4
    ("CruiseSpeed5", "130"),  # 巡航记忆速度 5
    ("PaddleMode", "0"),  # 方向盘拨片操作模式
    ("MyDrivingMode", "3"),  # 个人驾驶习惯模式 (1: 舒适, 2: 标准, 3: 激进)
    ("MyDrivingModeAuto", "0"),  # 自动切换驱动模式
    ("TrafficLightDetectMode", "2"),  # 红绿灯检测与响应模式
    ("CruiseMaxVals0", "160"),  # 巡航最大设定值 - 段 0
    ("CruiseMaxVals1", "200"),  # 巡航最大设定值 - 段 1
    ("CruiseMaxVals2", "160"),  # 巡航最大设定值 - 段 2
    ("CruiseMaxVals3", "130"),  # 巡航最大设定值 - 段 3
    ("CruiseMaxVals4", "110"),  # 巡航最大设定值 - 段 4
    ("CruiseMaxVals5", "95"),  # 巡航最大设定值 - 段 5
    ("CruiseMaxVals6", "80"),  # 巡航最大设定值 - 段 6
    ("LongTuningKpV", "100"),  # 纵向调优 P 增益 (百分比)
    ("LongTuningKiV", "0"),  # 纵向调优 I 增益 (百分比)
    ("LongTuningKf", "100"),  # 纵向调优 F 增益 (百分比)
    ("LongActuatorDelay", "20"),  # 纵向执行机构延迟补偿
    ("VEgoStopping", "50"),  # 车速判定为停止的阈值
    ("RadarReactionFactor", "100"),  # 雷达响应敏感度系数
    ("EnableRadarTracks", "0"),  # 开启原厂雷达目标点透显
    ("RadarLatFactor", "0"),  # 雷达侧向位置修正系数
    ("EnableEscc", "0"),  # 开启增强型 SCC 逻辑 (ESCC)
    ("EnableCornerRadar", "0"),  # 开启角雷达支持
    ("HyundaiCameraSCC", "0"),  # 现代相机 SCC 系统模式
    ("IsLdwsCar", "0"),  # 是否为无 LKAS 按钮车辆
    ("CanfdHDA2", "0"),  # 启用 CAN-FD HDA2 协议支持
    ("CanfdDebug", "0"),  # 开启 CAN-FD 数据调试日志
    ("SoundVolumeAdjust", "100"),  # 全局系统音量大小百分比
    ("SoundVolumeAdjustEngage", "10"),  # 开启巡航时的音量淡入比例
    ("TFollowGap1", "110"),  # 一档跟车距离设置 (秒 x 100)
    ("TFollowGap2", "120"),  # 二档跟车距离设置 (秒 x 100)
    ("TFollowGap3", "140"),  # 三档跟车距离设置 (秒 x 100)
    ("TFollowGap4", "160"),  # 四档跟车距离设置 (秒 x 100)
    ("DynamicTFollow", "1"),  # 开启动态跟车距离调整
    ("AChangeCostStarting", "10"),  # 起步时的加速度变化代价 (平顺度限制)
    ("TrafficStopDistanceAdjust", "400"),  # 红绿灯前停止距离偏置
    ("DynamicTFollowLC", "100"),  # 变道时的动态跟车系数
    ("HapticFeedbackWhenSpeedCamera", "0"),  # 测速相机报警时开启震动反馈
    ("UseLaneLineSpeed", "0"),  # 参考车道线曲率调节车速
    ("PathOffset", "0"),  # 路径侧向偏移调整
    ("UseLaneLineCurveSpeed", "0"),  # 弯道中根据车道线预测降速
    ("AdjustLaneOffset", "0"),  # 周期性调整车道位置
    ("AdjustCurveOffset", "15"),  # 弯道偏移修正
    ("LaneChangeNeedTorque", "0"),  # 换道是否需要手动介入扭矩触发
    ("LaneChangeDelay", "0"),  # 自动换道前的延迟确认时间
    ("LaneChangeBsd", "1"),  # 变道盲区监测开关
    ("MaxAngleFrames", "89"),  # 大转角保持最大允许帧数
    ("LateralTorqueCustom", "1"),  # 启用自定义扭矩控制参数 (忽略预设)
    ("LateralTorqueAccelFactor", "2800"),  # 侧向扭矩加速度增益系数
    ("LateralTorqueFriction", "100"),  # 侧向扭矩摩擦力静态补偿
    ("LateralTorqueKpV", "90"),  # 侧向扭矩 P 比例增益
    ("LateralTorqueKiV", "10"),  # 侧向扭矩 I 积分增益
    ("LateralTorqueKf", "100"),  # 侧向扭矩 FF 前馈增益
    ("LateralTorqueKd", "0"),  # 侧向扭矩 D 微分增益
    ("LatMpcPathCost", "350"),  # 侧向 MPC 控制路径代价
    ("LatMpcMotionCost", "7"),  # 侧向 MPC 控制运动代价
    ("LatMpcAccelCost", "120"),  # 侧向 MPC 加速度代价系数
    ("LatMpcJerkCost", "4"),  # 侧向 MPC Jerk 代价系数
    ("LatMpcSteeringRateCost", "7"),  # 侧向 MPC 转向速率代价系数
    ("LatMpcInputOffset", "4"),  # 侧向 MPC 路径输入偏置
    ("CustomSteerMax", "0"),  # 自定义最大转向扭矩上限
    ("CustomSteerDeltaUp", "0"),  # 自定义最大增扭速率
    ("CustomSteerDeltaDown", "0"),  # 自定义最大减扭速率
    ("CustomSteerDeltaUpLC", "3"),  # 变道时自定义最大增扭速率
    ("CustomSteerDeltaDownLC", "5"),  # 变道时自定义最大减扭速率
    ("SpeedFromPCM", "2"),  # 巡航速度来源 (1: 车辆仪表, 2: openpilot)
    ("SteerActuatorDelay", "25"),  # 转向执行延迟补偿时间
    ("LatSmoothSec", "18"),  # 横向平滑处理时长系数
    ("MaxTimeOffroadMin", "60"),  # 无操作后最大休眠前运行时长 (分)
    ("DisableDM", "1"),  # 禁用驾驶员监控（慎用）
    ("EnableConnect", "0"),  # 启用系统联网连接
    ("MuteDoor", "0"),  # 屏蔽车门未关报警
    ("MuteSeatbelt", "0"),  # 屏蔽安全带未系报警
    ("RecordRoadCam", "0"),  # 循环记录前方路面视频
    ("HDPuse", "0"),  # 使用高精度感知增强 (HDP)
    ("CruiseOnDist", "400"),  # 距离检测开启巡航距离
    ("UseUsbCamera", "1"),  # 默认使用 C++ 摄像头驱动
    ("HotspotOnBoot", "0"),  # 开机自动开启 WiFi 热点
    ("SoftwareMenu", "1"),  # 开启软件菜单显示
    ("CustomSR", "145"),  # 自定义转向比 (14.5)
    ("SteerRatioRate", "100"),  # 动态转向比调节速率
    ("NNFF", "1"),  # 启用神经网络前馈控制 (NNFF)
    ("NNFFLite", "0"),  # 启用轻量级神经网络前馈
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
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


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

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  print_timer = 0

  started_prev = False

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

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print_timer = (print_timer + 1)%10
    if print_timer == 0:
      print(running)
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

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
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
