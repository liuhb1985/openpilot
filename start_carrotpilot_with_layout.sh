#!/usr/bin/env bash
# launch_pc.sh — carrot-wip PC 开发者启动器
#
# 在装有本地 AMD GPU（/dev/kfd）的 Linux 工作站上启动 manager 与 modeld。
# 与车机用 launch_chffrplus.sh 的差异：
#   - 不写 /data/params、/persist/、/data/pythonpath；PC 模式下由
#     openpilot/system/hardware/hw.py:Paths 自动重定向到 ~/.comma。
#   - 不跑 AGNOS 更新、abctl、网络/SSH 配置。
#   - 不需要 sudo；普通用户身份运行。
#   - SCons 缓存目录回落到 /tmp/scons_cache（/data 通常不可写）。
#   - 大模型路径统一交给 helpers.usbgpu_present() 自动判定；本地 AMD
#     会直接走 big-model 编译/加载分支。
#
# 用法：
#   ./launch_pc.sh                 # 缺省：前台启动 manager
#   ./launch_pc.sh --tmux          # 起一个名为 op 的 tmux 会话
#   ./launch_pc.sh --rebuild-models # 强制重建 modeld artifacts
#   ./launch_pc.sh --no-manager    # 只准备环境与 modeld，不启动 manager
#   FORCE_BIG=1 ./launch_pc.sh     # 强制编译/加载大模型
#
# 可调环境变量：
#   PYTHON_BIN     解释器路径（默认 .venv/bin/python）
#   FORCE_BIG      设为 1 时即使 helpers.usbgpu_present() 为 False 也按 big 处理
#   LOG_ROOT       日志根目录（默认 /tmp/openpilot_pc）
#   NO_TMUX        设为 1 强制前台运行（脚本自动检测）
#   PREPAREONLY    传给 manager.py：只准备不启动业务循环
#   FINGERPRINT    车辆指纹默认值（默认 HYUNDAI_SANTA_FE，换车时覆盖）
#   SKIP_FW_QUERY  设为 1 跳过固件版本查询（默认 1）
#   ROAD_CAM_ID_PATTERN   主摄（长焦）by-id 匹配串（默认 RYS_RGB）
#   WIDE_CAM_ID_PATTERN   广角 by-id 匹配串（默认 LRCP_500W）
#   ROAD_CAM / WIDE_CAM   显式指定设备路径或编号时优先使用
#   WEBCAM_ROTATE180      设为 1 恢复 180° 旋转（摄像头倒装时使用；默认不旋转）
#   WEBCAM_FPS            采集帧率（默认 20；相机能出更高帧时可设 25~30）
#   WEBCAM_FOURCC         采集像素格式（Linux 默认 MJPG，避免 YUYV 把 USB 2.0 总线占满）
#   WEBCAM_EXPOSURE       手动曝光值（如 120）；不设则用自动曝光。自动曝光在室内会
#                         拉长曝光时间把帧率压到约 18fps，手动曝光可回到约 28fps，
#                         但无法适应隧道/夜间光线，行车时慎用
#   WEBCAM_AUTO_EXPOSURE  配合上项，曝光模式（默认 1=手动；0/3=自动）
#   FPS                   UI 渲染帧率（默认 20；GPU 够快可设 60）
#   NO_LAYOUT             设为 1 禁用启动后的窗口布局守护
#   LAYOUT_UI_PERCENT     UI 主窗口占屏高百分比（默认 70）
#   CAMERA_WINDOW_WIDTH / CAMERA_WINDOW_HEIGHT  盲区摄像头窗口尺寸（默认 640x360）
#   USE_WEBCAM      缺省自动置 1（dst 启动命令即 USE_WEBCAM=1）；NO_WEBCAM=1 可显式关闭
#   DISABLE_MSAA    缺省 1（PC 上关掉 4x MSAA，避免 UI 掉到个位数帧率）
#   BLOCK           传给 manager.py 的进程屏蔽表；检测不到 CH347 IMU 时自动追加 sensord
#   DRIVER_CAM      指定驾驶员摄像头时保留 DM；否则首次运行自动把 DisableDM 播种为 1

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-$ROOT/.venv/bin/python}"
LOG_ROOT="${LOG_ROOT:-/tmp/openpilot_pc}"
SCONS_CACHE_DIR="${SCONS_CACHE_DIR:-/tmp/scons_cache}"
OPENPILOT_PREFIX="${OPENPILOT_PREFIX:-}"
USE_TMUX=0
REBUILD_MODELS=0
RUN_MANAGER=1

while [ $# -gt 0 ]; do
  case "$1" in
    --tmux)      USE_TMUX=1 ;;
    --rebuild-models) REBUILD_MODELS=1 ;;
    --no-manager) RUN_MANAGER=0 ;;
    -h|--help)
      sed -n '2,28p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

if [ -n "${NO_TMUX:-}" ]; then USE_TMUX=0; fi

# ---------- 日志目录 ----------
mkdir -p "$LOG_ROOT" "$LOG_ROOT/tmux" "$SCONS_CACHE_DIR"

# ---------- 通用环境 ----------
# 单线程 BLAS；多线程会拖慢实时循环，反而拖累帧率。
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# PC 上不存在 QCOM hexagon DSP，这些标志在 launch_env.sh 里只是给车机用，
# 但保留无害。这里明确清零，避免影响 tinygrad 的 CPU 后端选择。
unset QCOM_PRIORITY

# 默认走 comma three 的 2160x1080 主界面布局（MiciMainLayout 536x240 在 PC
# 显示器上几乎不可见，且 application._calculate_auto_scale 在窗口小于屏幕
# 时直接返回 1.0，不会自动放大）。调用方想强制 mici 布局就 BIG=0 ./launch_pc.sh。
export BIG="${BIG:-1}"

# cereal 路径解析依赖 OPENPILOT_PREFIX；未设置时 Python 侧的
# os.environ.get("OPENPILOT_PREFIX", "") 与空串等价，因此只在非空时 export。
# 注意：绝不能 export 空字符串——msgq 的 C 层用 getenv() 判断，空串不是 NULL，
# 会把所有 endpoint 路径拼成 /dev/shm/msgq_/<endpoint> 目录结构，导致
# pub/sub 全部 "No such file or directory"。
if [ -n "${OPENPILOT_PREFIX:-}" ]; then
  export OPENPILOT_PREFIX
fi

# pydeps 是 launch_chffrplus.sh 给车机放离线依赖的目录；PC 上 .venv 已经
# 装好，PYDEPS 不创建也不引用。
export PYDEPS="$ROOT/pydeps"
mkdir -p "$PYDEPS" 2>/dev/null || PYDEPS="$ROOT/.pydeps"
mkdir -p "$PYDEPS"

# 车辆指纹默认值（参考 start_carrotpilot_with_layout.sh:55-56）。
# 固定指纹可跳过上车指纹识别；SKIP_FW_QUERY=1 跳过固件版本查询，
# 未插黑熊/开发调试时 card 不会卡在指纹流程。换车型时覆盖即可：
#   FINGERPRINT=KIA_SPORTAGE_5TH_GEN ./launch_pc.sh
export FINGERPRINT="${FINGERPRINT:-HYUNDAI_SANTA_FE}"
export SKIP_FW_QUERY="${SKIP_FW_QUERY:-1}"

# tinygrad 与 openpilot 必须早于系统 site-packages，避免命名冲突。
export PYTHONPATH="$ROOT/tinygrad_repo:$ROOT:${PYTHONPATH:-}"

# cereal 编译产物在 openpilot 目录下；scons 在 .venv 里。
export PATH="$ROOT/.venv/bin:$PATH"

# ---------- 基础自检 ----------
if [ ! -x "$PYTHON_BIN" ]; then
  echo "error: PYTHON_BIN ($PYTHON_BIN) not executable" >&2
  echo "  set PYTHON_BIN=... or run 'uv sync' in the repo root first" >&2
  exit 1
fi

if [ "$(uname -m)" != "x86_64" ]; then
  echo "warning: this launcher targets x86_64; detected $(uname -m)" >&2
fi

# /dev/kfd 在则本地 AMD 可用；否则 big-model 路径走 CPU fallback。
HAS_KFD=0
if [ -e /dev/kfd ]; then
  if "$PYTHON_BIN" -c 'import os; os.stat("/dev/kfd")' 2>/dev/null; then
    HAS_KFD=1
  fi
fi

echo "============================================================"
echo " carrot-wip PC launcher"
echo "   ROOT         = $ROOT"
echo "   PYTHON_BIN   = $PYTHON_BIN ($($PYTHON_BIN -V 2>&1))"
echo "   LOG_ROOT     = $LOG_ROOT"
echo "   SCONS_CACHE  = $SCONS_CACHE_DIR"
echo "   /dev/kfd     = $HAS_KFD"
echo "   FORCE_BIG    = ${FORCE_BIG:-0}"
echo "   tmux         = $USE_TMUX"
echo "============================================================"

# ---------- 系统级调优（参考 /home/peter/openpilot start_carrotpilot_with_layout.sh）----------
# 全部 best-effort：没有 sudo 权限或内核不支持时静默跳过，不阻断启动。
sys_tune() {
  # 1. CPU 高性能模式（降低调度延迟）
  echo performance | sudo -n tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
  # 2. 降低 swappiness，阻断实时循环内存换出
  sudo -n sysctl -w vm.swappiness=10 >/dev/null 2>&1 || true
  # 3. 加大本地套接字缓冲区，防 VisionIPC 视频流丢帧/绿屏
  sudo -n sysctl -w net.core.rmem_max=26214400 >/dev/null 2>&1 || true
  sudo -n sysctl -w net.core.wmem_max=26214400 >/dev/null 2>&1 || true
  # 4. I2C / USB 设备权限（sensord 与 USB 摄像头需要）
  sudo -n chmod 666 /dev/i2c-* >/dev/null 2>&1 || true
  [ -d /dev/bus/usb ] && sudo -n chmod -R 777 /dev/bus/usb >/dev/null 2>&1 || true
}
sys_tune

# ---------- CH347 IMU 自愈 ----------
# CH347（USB To UART+SPI+I2C）偶尔会进入“半挂死”状态：lsusb 能看到设备，但
# sysfs 下没有任何接口枚举，于是 mfd/i2c-ch347 不会创建 i2c 适配器，sensord 探测
# 不到 IMU 后回落去扫 GPU 的 /dev/i2c-*，刷屏 EIO。此时用 authorized 开关做一次
# 软重置即可恢复（不需要拔插）。
ensure_ch347_imu() {
  command -v udevadm >/dev/null 2>&1 || return 0

  local usb_dir name
  for usb_dir in /sys/bus/usb/devices/*; do
    [ -r "$usb_dir/idVendor" ] || continue
    [ "$(cat "$usb_dir/idVendor" 2>/dev/null)" = "1a86" ] || continue
    case "$(cat "$usb_dir/idProduct" 2>/dev/null)" in 55db|55dd) ;; *) continue ;; esac

    # 适配器名字里带 ch347 说明驱动已经绑定，无需处理
    name=""
    for i2c_dir in /sys/bus/i2c/devices/i2c-*; do
      [ -r "$i2c_dir/name" ] || continue
      if grep -qi ch347 "$i2c_dir/name" 2>/dev/null; then
        name="$(cat "$i2c_dir/name")"
        break
      fi
    done
    if [ -n "$name" ]; then
      echo "[ch347] IMU adapter present: $name"
      return 0
    fi

    echo "[ch347] device $(basename "$usb_dir") present but no i2c adapter; resetting USB"
    sudo -n sh -c "echo 0 > $usb_dir/authorized" >/dev/null 2>&1 || true
    sleep 1
    sudo -n sh -c "echo 1 > $usb_dir/authorized" >/dev/null 2>&1 || true
    sleep 2
    sudo -n modprobe mfd-ch347 >/dev/null 2>&1 || true
    for i2c_dir in /sys/bus/i2c/devices/i2c-*; do
      [ -r "$i2c_dir/name" ] || continue
      if grep -qi ch347 "$i2c_dir/name" 2>/dev/null; then
        echo "[ch347] IMU adapter recovered: $(cat "$i2c_dir/name")"
        return 0
      fi
    done
    echo "[ch347] WARNING: no i2c adapter after reset; sensord will report IMU errors"
    return 0
  done
}
ensure_ch347_imu

# ---------- PC 录像清理 ----------
# 车机 deleter 的阈值在本机大容量盘上永不触发，启动前按段数/容量清理一次。
bash "$ROOT/scripts/pc_media_cleanup.sh" || true

# ---------- PC 默认开关（对齐 dst 工程的 PC 启动约定）----------
# 1) USE_WEBCAM：dst 的启动命令里硬编码 USE_WEBCAM=1；本脚本此前只在未设置时打印一行
#    NOTE，裸跑时 webcamerad 被禁用（process_config.py:148 enabled=WEBCAM），一个摄像头
#    进程都没有，UI 只画占位色。这里缺省置 1；要跑无相机场景用 NO_WEBCAM=1 显式关掉
#    （USE_WEBCAM=0 不行——process_config 判断的是 "is not None"）。
if [ -z "${USE_WEBCAM:-}" ] && [ "${NO_WEBCAM:-0}" != "1" ]; then
  export USE_WEBCAM=1
  echo "[pc] USE_WEBCAM defaulted to 1 (webcamerad enabled)"
fi

# 2) UI：1822x912 开 4x MSAA 会把 UI 拖到个位数帧率，PC 上缺省关掉。
#    （DISABLE_MSAA 是 carrot 比 dst 多出的开关，dst 硬编码 samples(16) 无法关。）
export DISABLE_MSAA="${DISABLE_MSAA:-1}"

# 3) sensord：process_config.py:156 在 PC 上被强制 enabled=True，但机器上没有 CH347
#    USB-I2C 桥时 detect_ch347_backend() 返回 None，sensord 会直接退出、被 manager
#    每个 tick 重启。dst 用 not PC + IMU_SEL 专用进程规避；这里改用 manager 自带的
#    BLOCK 环境变量（manager.py:168）在没有 IMU 时屏蔽它，不改动 process_config。
pc_has_imu() {
  ls /dev/ch34x_pis* >/dev/null 2>&1 && return 0
  local d
  for d in /sys/bus/i2c/devices/i2c-*; do
    [ -r "$d/name" ] || continue
    grep -qi ch347 "$d/name" 2>/dev/null && return 0
  done
  return 1
}
if ! pc_has_imu; then
  echo "[pc] no CH347 IMU detected; appending sensord to BLOCK"
  BLOCK="${BLOCK:+$BLOCK,}sensord"
fi
if [ -n "${BLOCK:-}" ]; then
  export BLOCK
  echo "[pc] BLOCK=$BLOCK"
fi

# ---------- 摄像头就绪检测 + by-id 物理绑定（参考 start_carrotpilot_with_layout.sh:86-133）----------
# WEBCAM 模式下 webcamerad 需要 /dev/videoN；先等设备枚举稳定（最多 10s），
# 再用 /dev/v4l/by-id 按 USB 序列号绑定主/广角摄像头，插拔换口后编号不再漂移。
# 仅提示不阻断——没有摄像头时其他进程照常启动。
if [ -z "${USE_WEBCAM:-}" ]; then
  echo "[camera] NOTE: USE_WEBCAM is unset; webcamerad stays disabled (use 'USE_WEBCAM=1 $0')"
fi
if [ -n "${USE_WEBCAM:-}" ]; then
  cam_wait=0
  cam_count=0
  while [ "$cam_wait" -lt 10 ]; do
    cam_count=$({ ls /dev/video* 2>/dev/null || true; } | wc -l)
    if [ "$cam_count" -gt 0 ]; then
      echo "[camera] found $cam_count video device(s): $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
      break
    fi
    sleep 1
    cam_wait=$((cam_wait + 1))
  done
  if [ "$cam_count" = "0" ]; then
    echo "[camera] WARNING: USE_WEBCAM set but no /dev/video* found; webcamerad will fail" >&2
  fi

  # 按用途绑定一个摄像头：显式指定 > by-id 序列号 > by-path 备用 > 缺省编号。
  # $1=环境变量名(ROAD_CAM/WIDE_CAM) $2=by-id 匹配串 $3=by-path 备用匹配串 $4=缺省编号
  bind_cam() {
    local var="$1" id_pat="$2" path_pat="$3" default_idx="$4" dev
    if [ -n "${!var:-}" ]; then
      echo "[camera] $var explicitly set: ${!var}"
      return 0
    fi
    dev=$(ls /dev/v4l/by-id/*${id_pat}*video-index0 2>/dev/null | head -n 1 || true)
    if [ -n "$dev" ] && [ -e "$dev" ]; then
      echo "[camera] $var bound by-id ($id_pat): $dev"
      export "$var=$dev"
      return 0
    fi
    if [ -n "$path_pat" ]; then
      dev=$(ls /dev/v4l/by-path/*${path_pat}*video-index0 2>/dev/null | head -n 1 || true)
      if [ -n "$dev" ] && [ -e "$dev" ]; then
        echo "[camera] $var bound by-path fallback: $dev"
        export "$var=$dev"
        return 0
      fi
    fi
    echo "[camera] WARNING: $var ($id_pat) not found; falling back to index $default_idx" >&2
    export "$var=$default_idx"
  }

  bind_cam ROAD_CAM "${ROAD_CAM_ID_PATTERN:-RYS_RGB}" "${ROAD_CAM_BYPATH_PATTERN:-}" 0

  # 广角：匹配不到且未显式指定时不绑定，webcamerad 自动工作在单摄模式。
  wdev=$(ls /dev/v4l/by-id/*${WIDE_CAM_ID_PATTERN:-LRCP_500W}*video-index0 2>/dev/null | head -n 1 || true)
  if [ -n "${WIDE_CAM:-}" ]; then
    echo "[camera] WIDE_CAM explicitly set: ${WIDE_CAM}"
  elif [ -n "$wdev" ] && [ -e "$wdev" ]; then
    echo "[camera] WIDE_CAM bound by-id: $wdev"
    export WIDE_CAM="$wdev"
  else
    echo "[camera] wide cam (${WIDE_CAM_ID_PATTERN:-LRCP_500W}) not found; single-camera mode"
  fi
fi

# ---------- params_pyx.so ----------
# ensure_params_build.sh 已经做了 header sha256 → scons 增量构建判断，
# 缓存目录在 PC 上重定向到 /tmp/scons_cache。直接复用它。
if ! bash "$ROOT/scripts/ensure_params_build.sh" >"$LOG_ROOT/params_build.log" 2>&1; then
  echo "params_pyx.so build failed; see $LOG_ROOT/params_build.log" >&2
  tail -n 40 "$LOG_ROOT/params_build.log" >&2
  exit 1
fi

# ---------- PC 参数默认值 ----------
# PC 上没有红外驾驶员摄像头，但 carrot 的 DisableDM 未设置时 get_int 返回 0（=DM 开）。
# 一旦 USE_WEBCAM=1，就会连带启用 dmonitoringmodeld / dmonitoringd
# （process_config.py:155,168 的 enabled=(WEBCAM or not PC)），它们会卡在等一个永远
# 不会到来的 driver 流上，dmonitoringd 也拿不到 driverStateV2。
# dst 的默认参数就是 DisableDM=1 且启动脚本传 NO_DM=1，这里对齐它的行为：
# 只在参数从未设置时播种，不覆盖用户在设置界面里的选择；显式给了 DRIVER_CAM 则保留 DM。
if [ -z "${DRIVER_CAM:-}" ]; then
  "$PYTHON_BIN" - <<'PY' || true
from openpilot.common.params import Params
p = Params()
if p.get("DisableDM") is None:
    p.put("DisableDM", "1")
    print("[params] DisableDM defaulted to 1 (no driver camera on PC)")
PY
fi

# ---------- helpers 自检 ----------
echo "[helpers] probing tinygrad devices…"
DEVICES="$("$PYTHON_BIN" -c 'from tinygrad import Device; print(" ".join(Device.get_available_devices()))' 2>&1 | tail -n1)"
echo "[helpers] tinygrad devices: $DEVICES"

echo "[helpers] usbgpu_present check:"
"$PYTHON_BIN" - <<'PY' || true
from openpilot.selfdrive.modeld.helpers import usbgpu_present, usbgpu_compiled
present = usbgpu_present()
compiled = usbgpu_compiled()
print(f"  usbgpu_present = {present}")
print(f"  usbgpu_compiled = {compiled}")
PY

# ---------- modeld 产物 ----------
# 借鉴 launch_chffrplus.sh:invalidate_modeld_build_if_needed 的思路：
# 当 modeld/、tinygrad/、file_chunker.py 任一变动，或 driving_tinygrad.pkl
# 缺失时，强制 SCons 重建。需要保留 --rebuild-models 显式开关。
modeld_models_dir="$ROOT/openpilot/selfdrive/modeld/models"
driving_pkl="$modeld_models_dir/driving_tinygrad.pkl"
driving_manifest="$driving_pkl.chunkmanifest"
tg_devices_json="$modeld_models_dir/tg_input_devices.json"

if [ "$REBUILD_MODELS" = "1" ]; then
  echo "[models] --rebuild-models: removing driving_tinygrad.pkl*"
  rm -f "$driving_pkl" "$driving_pkl".chunk* "$tg_devices_json"
fi

need_model_rebuild=0
if [ ! -f "$driving_pkl" ] && [ ! -f "$driving_manifest" ]; then
  need_model_rebuild=1
  echo "[models] driving_tinygrad.pkl missing; rebuild required"
fi

if [ "$need_model_rebuild" = "1" ]; then
  echo "[models] invoking scons for modeld…"
  cd "$ROOT"
  scons -u -j"$(nproc)" \
    --warn=no-deprecated \
    openpilot/selfdrive/modeld/models \
    >"$LOG_ROOT/scons_models.log" 2>&1 || {
      echo "modeld scons build failed; see $LOG_ROOT/scons_models.log" >&2
      tail -n 60 "$LOG_ROOT/scons_models.log" >&2
      exit 1
    }
fi

# 大模型构建默认由 manager.py:build.py 负责；这里只在前置条件已具备
# (FORCE_BIG=1 且编译产物缺失) 时手工触发，避免重复编译。
if [ "${FORCE_BIG:-0}" = "1" ]; then
  echo "[models] FORCE_BIG=1: ensuring big model artifacts…"
  "$PYTHON_BIN" -m openpilot.selfdrive.modeld.big_model --ensure-if-egpu \
    --network-wait-seconds 30 \
    >"$LOG_ROOT/big_model.log" 2>&1 || {
      echo "big model ensure-if-egpu returned non-zero; see $LOG_ROOT/big_model.log" >&2
    }
fi

# ---------- 窗口布局守护（移植自 dst 工程 start_carrotpilot_with_layout.sh:180-312）----------
# 启动后自动拼接窗口铺满屏幕：UI 主窗口铺上半屏；左下为盲区摄像头窗口
# （carrot 暂无此窗口，出现即自动接管）；右下为本终端日志窗口。
# 与 dst 工程差异：按 xrandr 实际分辨率自适应（dst 硬编码 2048x1536），
# 摄像头窗口缺失时不必等满 30s。需要 X11 + wmctrl，无图形环境自动跳过。
start_layout_daemon() {
  if [ -n "${NO_LAYOUT:-}" ]; then
    echo "[layout] NO_LAYOUT set, skipping"
    return 0
  fi
  command -v wmctrl >/dev/null 2>&1 || { echo "[layout] wmctrl 不可用，跳过窗口布局"; return 0; }
  [ -n "${DISPLAY:-}" ] || { echo "[layout] 无 DISPLAY，跳过窗口布局"; return 0; }

  # 屏幕尺寸自适应
  local geom screen_w screen_h
  geom=$(xrandr 2>/dev/null | grep -w connected | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -n1 | cut -d+ -f1)
  screen_w="${geom%%x*}"
  screen_h="${geom##*x}"
  case "$screen_w$screen_h" in ''|*[!0-9]*) screen_w=1600; screen_h=1200 ;; esac

  local ui_h cam_w cam_h log_x log_w log_h
  ui_h=$(( screen_h * ${LAYOUT_UI_PERCENT:-70} / 100 ))
  [ "$ui_h" -lt 240 ] && ui_h=240
  cam_w="${CAMERA_WINDOW_WIDTH:-640}"
  cam_h="${CAMERA_WINDOW_HEIGHT:-360}"
  log_x="$cam_w"
  log_w=$(( screen_w - cam_w ))
  [ "$log_w" -lt 320 ] && log_w=320

  echo "[layout] screen=${screen_w}x${screen_h}  UI=${screen_w}x${ui_h}  cam=${cam_w}x${cam_h}  log=${log_w}x$((screen_h - ui_h))@${log_x},${ui_h}"

  # 设置终端标题，便于 wmctrl 精准定位日志窗口（dst 同款）
  echo -ne "\033]0;Log Window\007"

  (
    # 守护进程不能继承主脚本的 set -euo pipefail：探测不到窗口时 grep 返回 1，
    # pipefail 会让子壳在第一轮就退出（dst 工程未开这些选项，无此问题）。
    set +e
    set +u
    set +o pipefail

    local elapsed=0 term_win_id="" aligned_ui="" aligned_cam="" aligned_term=""

    # 沿进程树向上找当前终端窗口 ID（dst 同款算法）
    if [ -n "${WINDOWID:-}" ]; then
      term_win_id="$WINDOWID"
    else
      local pid=$$
      while [ "$pid" -gt 1 ]; do
        term_win_id=$(wmctrl -lp 2>/dev/null | grep " $pid " | head -n 1 | cut -d' ' -f1)
        [ -n "$term_win_id" ] && break
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -z "$pid" ] && break
      done
    fi
    [ -n "$term_win_id" ] && echo "[layout] terminal window: $term_win_id"

    # 持续维护布局最多 30s（0.5s 间隔），等 UI 窗口慢慢起来
    while [ "$elapsed" -lt 60 ]; do
      # 1. UI 主窗口
      local ui_win
      ui_win=$(wmctrl -l 2>/dev/null | grep -Ei "openpilot|MainWindow|\bui\b" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
      if [ -n "$ui_win" ] && [ "$ui_win" != "$aligned_ui" ]; then
        wmctrl -i -r "$ui_win" -b remove,fullscreen,maximized_horz,maximized_vert
        wmctrl -i -r "$ui_win" -b remove,decorations
        command -v xprop >/dev/null 2>&1 && \
          xprop -id "$ui_win" -f _MOTIF_WM_HINTS 32c -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" 2>/dev/null
        wmctrl -i -r "$ui_win" -e 0,0,0,"$screen_w","$ui_h"
        aligned_ui="$ui_win"
        echo "[layout] UI window aligned: $ui_win"
      fi

      # 动态计算 UI 实际底边，作为下方窗口的起始 Y（dst 智能对齐算法）
      local target_y="$ui_h"
      if [ -n "$aligned_ui" ]; then
        local ui_geom actual_y actual_h
        ui_geom=$(wmctrl -l -G 2>/dev/null | grep -i "$aligned_ui" | head -n 1)
        actual_y=$(echo "$ui_geom" | awk '{print $4}')
        actual_h=$(echo "$ui_geom" | awk '{print $6}')
        if [ -n "$actual_y" ] && [ "$actual_y" -eq "$actual_y" ] 2>/dev/null && [ -n "$actual_h" ]; then
          target_y=$(( actual_y + actual_h ))
        fi
      fi

      # 2. 盲区摄像头窗口（carrot 暂无，出现即接管）
      local cam_win
      cam_win=$(wmctrl -l 2>/dev/null | grep -Ei "All Cameras|camera|cameras|Camera 0" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
      if [ -n "$cam_win" ] && [ "$cam_win" != "$aligned_cam" ]; then
        wmctrl -i -r "$cam_win" -b remove,fullscreen,maximized_horz,maximized_vert
        wmctrl -i -r "$cam_win" -b remove,decorations
        command -v xprop >/dev/null 2>&1 && \
          xprop -id "$cam_win" -f _MOTIF_WM_HINTS 32c -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" 2>/dev/null
        wmctrl -i -r "$cam_win" -e 0,0,"$target_y","$cam_w","$cam_h"
        aligned_cam="$cam_win"
        echo "[layout] camera window aligned: $cam_win"
      fi

      # 3. 日志终端窗口
      local active_term_id="$term_win_id"
      if [ -z "$active_term_id" ]; then
        active_term_id=$(wmctrl -l 2>/dev/null | grep -Ei "Log Window" | head -n 1 | cut -d' ' -f1)
      fi
      if [ -z "$active_term_id" ]; then
        active_term_id=$(wmctrl -l 2>/dev/null | grep -Ei "terminal|bash" | head -n 1 | cut -d' ' -f1)
      fi
      if [ -n "$active_term_id" ] && [ "$active_term_id" != "$aligned_term" ]; then
        wmctrl -i -r "$active_term_id" -b remove,fullscreen,maximized_horz,maximized_vert
        wmctrl -i -r "$active_term_id" -e 0,"$log_x","$target_y","$log_w","$(( screen_h - ui_h ))"
        aligned_term="$active_term_id"
        echo "[layout] log window aligned: $active_term_id"
      fi

      # UI + 日志对齐即达成主要目标；摄像头窗口再等 10s，未出现就收工
      if [ -n "$aligned_ui" ] && [ -n "$aligned_term" ]; then
        if [ -n "$aligned_cam" ] || [ "$elapsed" -ge 20 ]; then
          echo "[layout] layout established, daemon exiting"
          break
        fi
      fi

      sleep 0.5
      elapsed=$((elapsed + 1))
    done
  ) &
  echo "[layout] guardian daemon started (pid $!)"
}

# ---------- 启动 manager ----------
if [ "$RUN_MANAGER" = "0" ]; then
  echo "[manager] --no-manager: skipping manager startup"
  exit 0
fi

# ---------- 防双启动：启动前清掉旧的 manager 栈 ----------
# manager.py 的 unblock_stdout() 会 forkpty 出真正跑业务循环的子进程；旧栈未退净时
# 再次启动，新栈的 carrot_man(7710/7713)、carrot_navi(7712/7714)、amap_navi(8088/UDP)
# 会因端口被占在线程内直接崩溃，而进程本身不退出，manager 的 restart_if_crash 只能
# 感知进程级退出，无法自愈。这里先停掉旧栈并等关键端口释放再启动。
stop_old_stack() {
  if [ -n "${ALLOW_MULTI_MANAGER:-}" ]; then
    echo "[guard] ALLOW_MULTI_MANAGER set; skipping old-stack cleanup"
    return 0
  fi

  local old_pids
  # 注意脚本有 set -euo pipefail：pgrep 无匹配返回 1，管道会让赋值失败，
  # 因此先单独取值再自行换行转空格。
  old_pids="$(pgrep -f "system/manager/manager\.py" 2>/dev/null || true)"
  old_pids="${old_pids//$'\n'/ }"
  if [ -n "$old_pids" ]; then
    echo "[guard] stopping old manager: $old_pids"
    # SIGINT 让 manager 走 manager_cleanup；relay 父进程也会把信号转发给 pty 子进程
    kill -INT $old_pids 2>/dev/null || true
  fi

  # 旧 manager 被强杀（如终端关闭 SIGHUP 只打到 relay）时，其子服务会变孤儿继续
  # 占端口/摄像头。按“工作目录在本仓库内”识别遗留服务进程（含 openpilot.tools.*
  # 的 webcamerad），一并 SIGINT。
  local pid cwd
  for pid in $(pgrep -u "$(id -u)" -f "openpilot\.(selfdrive|system|tools)\." 2>/dev/null); do
    [ -d "/proc/$pid" ] || continue
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    case "$cwd" in
      "$ROOT"*) echo "[guard] stopping leftover service pid $pid"; kill -INT "$pid" 2>/dev/null || true ;;
    esac
  done
  for pid in $(pgrep -u "$(id -u)" -x pandad 2>/dev/null; pgrep -u "$(id -u)" -x loggerd 2>/dev/null; pgrep -u "$(id -u)" -x encoderd 2>/dev/null); do
    [ -d "/proc/$pid" ] || continue
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    case "$cwd" in
      "$ROOT"*) echo "[guard] stopping leftover native service pid $pid"; kill -INT "$pid" 2>/dev/null || true ;;
    esac
  done

  # 等待退出，最多 15s，超时 SIGKILL
  local waited=0
  while [ "$waited" -lt 30 ]; do
    pgrep -f "system/manager/manager\.py" >/dev/null 2>&1 || break
    sleep 0.5; waited=$((waited + 1))
  done
  if pgrep -f "system/manager/manager\.py" >/dev/null 2>&1; then
    echo "[guard] old manager still alive; sending SIGKILL"
    pkill -KILL -f "system/manager/manager\.py" 2>/dev/null || true
    sleep 1
  fi

  # 进程退出后端口释放仍可能有延迟；等关键端口全部空闲，最多 10s
  local ports="7000 7710 7712 7713 7714 8088 8888" p busy="" waited_ports=0
  while [ "$waited_ports" -lt 20 ]; do
    busy=""
    for p in $ports; do
      ss -H -ltn "sport = :$p" 2>/dev/null | grep -q . && busy="$busy $p"
    done
    [ -z "$busy" ] && break
    sleep 0.5; waited_ports=$((waited_ports + 1))
  done
  if [ -n "$busy" ]; then
    echo "[guard] WARNING: ports still busy after old-stack exit:$busy"
  fi
}
stop_old_stack

cd "$ROOT/openpilot/system/manager"
echo "[manager] starting…  (logs: $LOG_ROOT/manager.log)"

# 布局守护与 manager 并行启动：UI 窗口由 manager 拉起后由守护自动拼接
start_layout_daemon

if [ "$USE_TMUX" = "1" ] && command -v tmux >/dev/null 2>&1; then
  SESSION="op${OPENPILOT_PREFIX}"
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[tmux] session '$SESSION' exists; attaching"
    exec tmux attach -t "$SESSION"
  fi
  TMUX_LOG="$LOG_ROOT/tmux/manager.log"
  tmux new-session -d -s "$SESSION" -x 220 -y 50 \
    "\"$PYTHON_BIN\" \"$ROOT/openpilot/system/manager/manager.py\" 2>&1 | tee \"$TMUX_LOG\""
  echo "[tmux] new session: $SESSION (detach with Ctrl-b d)"
  exec tmux attach -t "$SESSION"
else
  if [ "$USE_TMUX" = "1" ]; then
    echo "[tmux] tmux not installed; running in foreground"
  fi
  exec "$PYTHON_BIN" "$ROOT/openpilot/system/manager/manager.py"
fi