#!/bin/bash

# 设置终端窗口标题为 "Log Window" 以便 wmctrl 精准定位
echo -ne "\033]0;Log Window\007"

# 设置 USB 权限
sudo chmod -R 777 /dev/bus/usb/*
sudo chmod 666 /dev/i2c-*

# 进入 MinerU 目录

# 设置环境变量并激活虚拟环境
export FINGERPRINT="HYUNDAI_SANTA_FE"
export SKIP_FW_QUERY="1"
source .venv/bin/activate

# 等待所有 USB 摄像头加载
check_cameras_ready() {
    echo "Wait for cameras to stabilize..."
    local expected=2
    local timeout=10
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        local count=$(ls /dev/video* 2>/dev/null | wc -l)
        if [ $count -ge $expected ]; then
             echo "✓ Found $count camera devices."
             export CAMERAS_EXIST=1
             return 0
        fi
        sleep 0.2
        elapsed=$((elapsed + 2))
        echo "Searching... ($count/$expected)"
    done

    local final_count=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ $final_count -gt 0 ]; then
        export CAMERAS_EXIST=1
    else
        export CAMERAS_EXIST=0
    fi
    echo "⚠ Warning: Expected $expected cameras but found $final_count. Proceeding anyway..."
}
check_cameras_ready
# -----------------------------------------------------------------------------
# 摄像头持久化路径配置 (解决 errno 25 / 绿屏问题)
# -----------------------------------------------------------------------------
# 1. 长焦摄像头 (Long-focus: RYS RGB Camera)
# 升级为终极的 by-id 物理绑定（永久解决插拔 USB 端口变动问题）
ROAD_PATH=$(ls /dev/v4l/by-id/*RYS_RGB*video-index0 2>/dev/null | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
    export ROAD_CAM=$ROAD_CAM_IDX
    echo "[INFO] Identified Road Cam (Long-focus) via ID $ROAD_PATH at index: $ROAD_CAM_IDX"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.4)
    ROAD_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.4:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
        ROAD_DEV=$(readlink -f "$ROAD_PATH")
        export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
        export ROAD_CAM=$ROAD_CAM_IDX
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
    export WIDE_CAM=$WIDE_CAM_IDX
    echo "[INFO] Tele-Camera Enabled: WIDE_PATH via ID $WIDE_PATH at index: $WIDE_CAM"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.2.3)
    WIDE_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.2.3:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
        WIDE_DEV=$(readlink -f "$WIDE_PATH")
        export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
        export WIDE_CAM=$WIDE_CAM_IDX
        echo "[INFO] Tele-Camera Enabled: WIDE_PATH via path $WIDE_PATH at index: $WIDE_CAM"
    else
        export WIDE_CAM=2
        export WIDE_CAM_IDX=2
        echo "[WARN] Wide Cam path not found, falling back to default index 2"
    fi
fi

# 启动前硬件预检查
wait_for_device() {
    local dev=$1
    local name=$2

    # 如果初始扫描完全没发现摄像头，或者设备路径未识别，则跳过长时间等待
    if [ "$CAMERAS_EXIST" = "0" ]; then
        echo "[SKIP] $name ($dev) skipped (no cameras detected in initial scan)"
        return 0
    fi

    echo "Waiting for $name ($dev)..."
    local timeout=2  # 缩短单项等待超时
    local elapsed=0
    while [ ! -e "$dev" ] && [ $elapsed -lt $timeout ]; do
        sleep 0.5
        elapsed=$((elapsed + 1))
    done
    if [ -e "$dev" ]; then
        echo "✓ $name found!"
    else
        echo "⚠ Warning: $name ($dev) not found after ${timeout}s"
    fi
}

wait_for_device "/dev/video$ROAD_CAM_IDX" "Road Camera (Long-focus)"
wait_for_device "/dev/video$WIDE_CAM_IDX" "Wide Camera (Wide-angle)"

# 设置动态库路径，使相关进程能找到 libonnxruntime
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/third_party/onnxruntime/lib

# 深度优化：开启 VisionIPC 模式，让盲区监测共享模型摄像头数据，消除硬件占用冲突
export USE_VISION_IPC=1

# 在后台启动 manager
echo "Starting openpilot manager with Dual-Camera Support..."
# 使用完整路径以配合 USE_CAMPATH=1，确保设备识别的唯一性
USE_WEBCAM=1 ROAD_CAM=$ROAD_PATH WIDE_CAM=$WIDE_PATH NO_DM=1 JY62=1 USE_CAMPATH=0 system/manager/manager.py &

MANAGER_PID=$!

# 等待监控任务完成
# 在后台启动系统管理器
# SCALE=1 USE_WEBCAM=1 USE_CAMPATH=1 python3 system/manager/manager.py &

# -----------------------------------------------------------------------------
# 窗口布局参数定义 (适配 2048x1536 高清屏幕 - 黄金分割对称分布)
# -----------------------------------------------------------------------------
# 1. 主界面窗口 (占据上半屏幕 2048x768)
UI_WINDOW_X=0
UI_WINDOW_Y=0
UI_WINDOW_WIDTH=2048
UI_WINDOW_HEIGHT=768

# 2. 盲区画面窗口 (占据左下半屏幕 1024x768)
CAMERA_WINDOW_WIDTH=1024
CAMERA_WINDOW_HEIGHT=768
CAMERA_0_X=0
CAMERA_0_Y=768
CAMERA_1_X=0
CAMERA_1_Y=768
CAMERA_2_X=0
CAMERA_2_Y=768

# 3. 日志命令窗口 (占据右下半屏幕 1024x768)
LOG_WINDOW_X=1024
LOG_WINDOW_Y=768
LOG_WINDOW_WIDTH=1024
LOG_WINDOW_HEIGHT=768

# -----------------------------------------------------------------------------
# 终极自适应窗口布局分配器 (支持大小写不敏感与并发异步捕获)
# -----------------------------------------------------------------------------

move_resize_window() {
    local pattern=$1
    local x=$2
    local y=$3
    local w=$4
    local h=$5
    local name=$6
    local win_id=""
    
    echo "Waiting for window: $name..."
    for i in {1..600}; do
        # 匹配标题且排除开发工具与无关终端
        win_id=$(wmctrl -l | grep -Ei "$pattern" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
        if [ ! -z "$win_id" ]; then
            echo "✓ Found window $name ($win_id), forcing layout ($x, $y, $w, $h)..."
            # 暴力长宽全屏/最大化重置
            wmctrl -i -r "$win_id" -b remove,fullscreen,maximized_horz,maximized_vert
            # 移除窗口边框修饰，实现像素级无缝拼接
            wmctrl -i -r "$win_id" -b remove,decorations
            
            # X11 异步缓冲让步：确保窗口管理器已完成状态重置
            sleep 0.3
            
            # 强力几何尺寸定位
            wmctrl -i -r "$win_id" -e 0,$x,$y,$w,$h
            # 移除强置顶状态，恢复优雅堆叠
            wmctrl -i -r "$win_id" -b remove,below,above
            
            # 【核心防抢占优化】：延迟 1.5 秒二次修正，打败 Qt/OpenCV 启动结束时的强制拉伸
            (sleep 1.5 && wmctrl -i -r "$win_id" -e 0,$x,$y,$w,$h) &
            return 0
        fi
        sleep 0.2
    done
    echo "⚠ Warning: Window $name ($pattern) not found after 120s"
    echo "=== [DEBUG DIAGNOSTIC] Current X11 Window List ==="
    wmctrl -l
    echo "=================================================="
    return 1
}

# 并发异步捕获各个窗口，防止启动先后顺序造成阻塞
move_resize_window "openpilot|MainWindow|\bui\b|^ui$" $UI_WINDOW_X $UI_WINDOW_Y $UI_WINDOW_WIDTH $UI_WINDOW_HEIGHT "ui" &
move_resize_window "camera|cameras|Camera 0|Camera 1|Camera 2" $CAMERA_0_X $CAMERA_0_Y $CAMERA_WINDOW_WIDTH $CAMERA_WINDOW_HEIGHT "all cameras" &

# 终极终端窗口定位：优先使用 $WINDOWID 环境变量进行高可靠物理绑定
if [ ! -z "$WINDOWID" ]; then
    (
        echo "✓ Found running terminal window via \$WINDOWID ($WINDOWID), positioning to log window layout..."
        wmctrl -i -r "$WINDOWID" -b remove,fullscreen,maximized_horz,maximized_vert
        wmctrl -i -r "$WINDOWID" -b remove,decorations
        sleep 0.3
        wmctrl -i -r "$WINDOWID" -e 0,$LOG_WINDOW_X,$LOG_WINDOW_Y,$LOG_WINDOW_WIDTH,$LOG_WINDOW_HEIGHT
        (sleep 1.5 && wmctrl -i -r "$WINDOWID" -e 0,$LOG_WINDOW_X,$LOG_WINDOW_Y,$LOG_WINDOW_WIDTH,$LOG_WINDOW_HEIGHT) &
    ) &
else
    # 备用方案：按标题搜索
    move_resize_window "log window|terminal|bash" $LOG_WINDOW_X $LOG_WINDOW_Y $LOG_WINDOW_WIDTH $LOG_WINDOW_HEIGHT "log window" &
fi

# 延迟等待系统全部就位，调整各层级
(
    sleep 4.0
    for pattern in "openpilot|MainWindow|\bui\b|^ui$" "camera|cameras" "log window|terminal|bash"; do
        win_id=$(wmctrl -l | grep -Ei "$pattern" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
        if [ ! -z "$win_id" ]; then
            wmctrl -i -r "$win_id" -b remove,below,above
            wmctrl -i -a "$win_id"
        fi
    done
) &

# 等待所有后台进程结束
wait