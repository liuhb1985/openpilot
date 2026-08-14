import os
import operator
import importlib.util

from cereal import car
from openpilot.common.params import Params
from openpilot.system.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess

FLASK_AVAILABLE = importlib.util.find_spec("flask") is not None

WEBCAM = os.getenv("USE_WEBCAM") is not None
USBCAM = os.getenv("USE_USBCAM") is not None
JY62 = int(os.getenv("JY62","2"))
NATIVE_LANE = os.getenv("USE_NATIVE_LANE") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def enable_updated(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started and params.get_bool("SoftwareMenu")

def check_fleet(started, params, CP: car.CarParams) -> bool:
  return FLASK_AVAILABLE

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

def enable_dm(started, params, CP: car.CarParams) -> bool:
  return (started or params.get_bool("IsDriverViewEnabled")) and params.get_int("DisableDM") == 0

def enable_connect(started, params, CP: car.CarParams) -> bool:
  return params.get_int("EnableConnect") > 0

def enable_xiaoge_data(started, params, CP: car.CarParams) -> bool:
  return params.get_bool("ShareData")

def c3x_lite(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("HardwareC3xLite")

procs = [
  # 云端后台管理进程，负责与 comma 平台进行通信
  DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid", enabled=not PC),

  # 视频流编码进程，负责处理摄像头画面的推流/录制
  # NativeProcess("loggerd", "system/loggerd", ["./loggerd"], logging),
  # NativeProcess("encoderd", "system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], notcar),
  
  # 日志消息处理进程，负责系统内部日志的分发
  PythonProcess("logmessaged", "system.logmessaged", always_run, enabled=not PC),

  # 摄像头驱动进程系列：支持 USB、Webcam 和原生 TICI 摄像头
  #NativeProcess("usbcamerad", "camera", ["./usb_camera"], driverview, enabled=USBCAM),
  NativeProcess("usbcamerad", "tools/webcam", ["./camerad"], driverview, enabled=USBCAM),
  NativeProcess("camerad", "system/camerad", ["./camerad"], driverview, enabled=not USBCAM and not WEBCAM),
  PythonProcess("webcamerad", "tools.webcam.camerad", driverview, enabled=WEBCAM and not USBCAM),
  
  # 系统状态记录：logcat 日志、进程资源消耗统计
  NativeProcess("logcatd", "system/logcatd", ["./logcatd"], only_onroad),
  NativeProcess("proclogd", "system/proclogd", ["./proclogd"], only_onroad),
  
  # 麦克风音频采集
  PythonProcess("micd", "system.micd", iscar),
  # 系统时间同步
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  # AI 核心进程：驾驶模型推理 (20Hz)
  PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),
  # 驾驶员监控模型推理
  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", enable_dm, enabled=(WEBCAM or not PC)),
  #NativeProcess("mapsd", "selfdrive/navd", ["./mapsd"], only_onroad),
  #NativeProcess("mapsd", "selfdrive/navd", ["./mapsd"], always_run),
  #PythonProcess("navmodeld", "selfdrive.modeld.navmodeld", only_onroad),
  PythonProcess("sensord", "system.sensord.sensord", only_onroad, enabled=not PC),
  # NativeProcess("sensord_jy62", "system/sensord", ["./sensord_jy62"], only_onroad, enabled=PC),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], only_onroad, enabled=PC),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], always_run, watchdog_max_dt=(5 if not PC else None)),
  NativeProcess("sensord_ch347", "system/sensord", ["./ch347t"], always_run, enabled=PC, watchdog_max_dt=3),
  #NativeProcess("ubuntu347t", "system/sensord", ["./ubuntu347t"], always_run, enabled=PC,watchdog_max_dt=3),
  # NativeProcess("sensord_jy62", "system/sensord", ["./sensord_jy62"], only_onroad, enabled=PC),
  PythonProcess("soundd", "selfdrive.ui.soundd", only_onroad),
  PythonProcess("locationd", "selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  
  # 摄像头在线校准进程 (计算俯仰角和偏航角)
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  # 车辆动力学估算 (扭矩学习、摩擦力补偿)
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  # 车辆控制核心 (执行 PID/MPC 控制算法，发布横纵向指令)
  PythonProcess("controlsd", "selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  # 摇杆调试驱动
  PythonProcess("joystickd", "tools.joystick.joystickd", or_(joystick, notcar)),
  # 自动驾驶状态机 (管理系统状态切换)
  PythonProcess("selfdrived", "selfdrive.selfdrived.selfdrived", only_onroad),
  # 车辆接口总线逻辑 (CAN 数据解析与打包)
  PythonProcess("card", "selfdrive.car.card", only_onroad),
  # 过期日志删除工具
  PythonProcess("deleter", "system.loggerd.deleter", always_run, enabled=not PC),
  # 驾驶员疲劳/注意力监控逻辑
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enable_dm, enabled=(WEBCAM or not PC)),
  # TICI/QCOM GPS 专用驱动
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  # 导航地图渲染与指令处理
  PythonProcess("navd", "selfdrive.navd.navd", only_onroad),
  # Panda 通讯与状态管理 (核心通讯链路)
  PythonProcess("pandad", "selfdrive.pandad.pandad", always_run),
  # 车辆参数在线学习 (轴距、转向比等)
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  # 通讯延迟估算
  PythonProcess("lagd", "selfdrive.locationd.lagd", only_onroad),
  
  # Ublox GPS 模块驱动 (TICI 专用)
  PythonProcess("ubloxd", "system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  
  # 路径规划核心 (处理纵向加速、跟车、减速)
  PythonProcess("plannerd", "selfdrive.controls.plannerd", not_long_maneuver),
  # 特殊纵向动作规划 (如变道辅助、绕行)
  PythonProcess("maneuversd", "tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  # 雷达点云处理进程
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  # 硬件抽象层 (管理风扇、电量、热状态)
  PythonProcess("hardwared", "system.hardware.hardwared", always_run),
  # 系统崩溃日志转储 (TICI 专用)
  PythonProcess("tombstoned", "system.tombstoned", always_run, enabled=not PC),
  # 系统自动更新进程
  PythonProcess("updated", "system.updated.updated", enable_updated, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", enable_connect),
  PythonProcess("statsd", "system.statsd", always_run),

  # 调试进程：消息桥接、WebRTC 远程画面、Web 摇杆
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", notcar),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),
  PythonProcess("joystick", "tools.joystick.joystick_control", and_(joystick, iscar)),

  #PythonProcess("fleet_manager", "selfdrive.frogpilot.fleetmanager.fleet_manager", check_fleet, enabled=not PC),
  PythonProcess("fleet_manager", "selfdrive.frogpilot.fleetmanager.fleet_manager", check_fleet),
  PythonProcess("carrot_man", "selfdrive.carrot.carrot_man", always_run),#, enabled=not PC),

  #Xiaoge data broadcaster (conditional on ShareData param)
  PythonProcess("xiaoge_data", "selfdrive.carrot.xiaoge_data", enable_xiaoge_data),
  # c3x lite
  PythonProcess("beep", "selfdrive.controls.beep", c3x_lite, enabled=TICI),
  NativeProcess("cam_blindspot", "camera", ["./cam_blindspot"], always_run, enabled=PC),
  PythonProcess("amap_navi", "selfdrive.carrot.amap_navi", always_run),
  PythonProcess("lane", "selfdrive.carrot.lane", only_onroad),
]

managed_processes = {p.name: p for p in procs}
