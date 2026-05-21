#!/bin/bash
# 禁用脚本开头的冗余硬件刷新 (如果设备没变，sudo udevadm 会拖慢启动)
# sudo udevadm control --reload-rules && sudo udevadm trigger
# sudo chmod 777 /dev/bus/usb/* &>/dev/null

export FINGERPRINT="HYUNDAI_SANTA_FE"
export SKIP_FW_QUERY="1"
export USE_NATIVE_LANE=1

# 屏蔽无关后台进程以实现“秒开”
BLOCK_PROCS="manage_athenad,uploader,statsd,logcatd,proclogd,deleter,loggerd,tombstoned,bootlog"

#sleep 1
# 运行准备
export GPU=1
export FORCE_GPU_BACKEND="OPENCL"

# -----------------------------------------------------------------------------
# 摄像头自愈与预检查函数
# -----------------------------------------------------------------------------
wait_for_device() {
    local dev="$1"
    local name="$2"
    echo "[WAIT] Waiting for $name ($dev) ..."
    for i in {1..15}; do
        if [ -e "$dev" ]; then
            echo "[OK] $name found."
            return 0
        fi
        sleep 0.5
    done
    echo "[WARN] $name ($dev) not found after timeout."
    return 1
}

source .venv/bin/activate &&
#USE_WEBCAM=1 ROAD_CAM=0 WIDE_CAM=2 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
#USE_USBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
# 获取屏幕尺寸 (manager 启动前先算好)
SCREEN_WIDTH=$(xwininfo -root | grep 'Width:' | awk '{print $2}')
SCREEN_HEIGHT=$(xwininfo -root | grep 'Height:' | awk '{print $2}')
SCREEN_WIDTH=${SCREEN_WIDTH:-1920}
SCREEN_HEIGHT=${SCREEN_HEIGHT:-1080}

# 布局设计（上下叠加，恢复比例）：
#  ┌─────────────────────────────────┐
#  │                                 │
#  │          openpilot UI           │  ← 占上方 2/3 (约 67%)
#  │                                 │
#  ├─────────────────────────────────┤
#  │          All Cameras            │  ← 占下方 1/3 (约 33%)
#  └─────────────────────────────────┘
# 布局设计 (PIP 模式 - 画中画)：
#  ┌─────────────────────────────────┐
#  │ [BSM View] (640x320) @ (0,0)    │  ← 缩小并移动到左上角
#  │                                 │
#  │          openpilot UI           │  ← 恢复全屏高度，背景显示
#  │                                 │
#  └─────────────────────────────────┘
UI_H=$SCREEN_HEIGHT
CAM_W_SMALL=640
CAM_H_SMALL=320

# ========================================================
# 方案：通过环境变量直接传递给 cam_blindspot 进程，由其内部调用 cv::moveWindow
# ========================================================
export CAM_X=0
export CAM_Y=0
export CAM_W=$CAM_W_SMALL
export CAM_H=$CAM_H_SMALL

# 确保视频目录存在
[ -d videos ] || mkdir -p videos

# 终极健壮的窗口定位函数 (仅用于 UI，摄像头现在自定位)
robust_place() {
    local key="$1"
    local class_key="$2"
    local x=$3 y=$4 w=$5 h=$6
    (
        local wid=""
        # echo "[wait] Searching for window ($key / class: $class_key) ..."

        for i in {1..200}; do # 增加循环次数到 200 (约 40 秒)
            # 移除 --onlyvisible 以防止某些映射中的窗口被跳过
            if [ -n "$class_key" ]; then
                wid=$(xdotool search --class "$class_key" 2>/dev/null | tail -n1)
            fi
            if [ -z "$wid" ]; then
                wid=$(xdotool search --name "$key" 2>/dev/null | tail -n1)
            fi
            # 兼容性备选搜索：搜索 openpilot
            if [ -z "$wid" ]; then
                wid=$(xdotool search --name "openpilot" 2>/dev/null | tail -n1)
            fi

            if [ -n "$wid" ]; then
                # echo "[info] Found window '$key', positioning..."
                # 连续锁定次数减少到 15 次 (约 3 秒)，足以让窗口稳定
                for j in {1..15}; do
                    wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz 2>/dev/null
                    xdotool windowmove "$wid" $x $y
                    xdotool windowsize "$wid" $w $h
                    sleep 0.2
                done
                return
            fi
            sleep 0.2 # 稍微增加等待间隔
        done

        echo "[error] TIMEOUT: Could not find '$key' or class '$class_key' after 40s"
        echo "Current windows list (wmctrl -l):"
        wmctrl -l
    ) &
}

# 监控 openpilot UI
robust_place "ui" "ui" 0 0 $SCREEN_WIDTH $UI_H

# -----------------------------------------------------------------------------
# 摄像头持久化路径配置 (解决 errno 25 / 绿屏问题)
# -----------------------------------------------------------------------------
# 1. 长焦摄像头 (Long-focus: RYS RGB Camera)
# 升级为终极的 by-id 物理绑定（永久解决插拔 USB 端口变动问题）
ROAD_PATH=$(ls /dev/v4l/by-id/*RYS_RGB*video-index0 2>/dev/null | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
    export ROAD_CAM=$ROAD_PATH
    echo "[INFO] Identified Road Cam (Long-focus) via ID $ROAD_PATH at index: $ROAD_CAM_IDX"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.4)
    ROAD_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.4:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
        ROAD_DEV=$(readlink -f "$ROAD_PATH")
        export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
        export ROAD_CAM=$ROAD_PATH
        echo "[INFO] Identified Road Cam (Long-focus) via path $ROAD_PATH at index: $ROAD_CAM_IDX"
    else
        export ROAD_CAM=0
        export ROAD_CAM_IDX=0
        echo "[WARN] Road Cam path not found, falling back to default index 0"
    fi
fi

# 2. 广角摄像头 (Wide-angle: LRCP 500W)
# 升级为终极的 by-id 物理绑定（永久解决插拔 USB 端口变动问题）
WIDE_PATH=$(ls /dev/v4l/by-id/*LRCP_500W*video-index0 2>/dev/null | head -n 1)
if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
    WIDE_DEV=$(readlink -f "$WIDE_PATH")
    export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
    export WIDE_CAM=$WIDE_PATH
    echo "[INFO] Tele-Camera Enabled: WIDE_PATH via ID $WIDE_PATH at index: $WIDE_CAM"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.2.3)
    WIDE_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.2.3:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
        WIDE_DEV=$(readlink -f "$WIDE_PATH")
        export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
        export WIDE_CAM=$WIDE_PATH
        echo "[INFO] Tele-Camera Enabled: WIDE_PATH via path $WIDE_PATH at index: $WIDE_CAM"
    else
        export WIDE_CAM=2
        export WIDE_CAM_IDX=2
        echo "[WARN] Wide Cam path not found, falling back to default index 2"
    fi
fi

# 启动前硬件预检查
wait_for_device "/dev/video$ROAD_CAM_IDX" "Road Camera (Long-focus)"
wait_for_device "/dev/video$WIDE_CAM_IDX" "Wide Camera (BSM)"

# 导出环境变量供 cam_blindspot 进程内部使用，确保路径 100% 同步
export WIDE_CAM_DEV="/dev/video$WIDE_CAM_IDX"

# 在前台启动 manager (开启精简模式)
USE_USBCAM=1 ROAD_CAM=$ROAD_CAM WIDE_CAM=$WIDE_CAM NO_DM=0 JY62=0 BLOCK=$BLOCK_PROCS system/manager/manager.py

# 等待监控任务完成