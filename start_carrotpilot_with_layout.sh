#!/bin/bash

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
             return 0
        fi
        sleep 0.2
        elapsed=$((elapsed + 2))
        echo "Searching... ($count/$expected)"
    done
    echo "⚠ Warning: Expected $expected cameras but found $count. Proceeding anyway..."
}
check_cameras_ready
# -----------------------------------------------------------------------------
# 摄像头持久化路径配置 (解决 errno 25 / 绿屏问题)
# -----------------------------------------------------------------------------
# 1. 长焦摄像头 (Long-focus, 默认 index 2)
# 使用通配符匹配，解决 usb-0 vs usbv2-0 或微小路径变动
ROAD_PATH=$(ls /dev/v4l/by-path/*usb*-0:2.3:1.0-video-index0 2>/dev/null | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
    export ROAD_CAM=$(basename "$ROAD_PATH")
    echo "[INFO] Identified Road Cam (Long-focus) via path $ROAD_PATH at index: $ROAD_CAM_IDX"
else
    export ROAD_CAM=2
    export ROAD_CAM_IDX=2
    echo "[WARN] Road Cam path not found, falling back to default index 2"
fi

# 2. 广角摄像头 (Wide-angle / BSM, 默认 index 0)
WIDE_PATH=$(ls /dev/v4l/by-path/*usb*-0:1:1.0-video-index0 2>/dev/null | head -n 1)
if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
    WIDE_DEV=$(readlink -f "$WIDE_PATH")
    export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
    export WIDE_CAM=$(basename "$WIDE_PATH")
    echo "[INFO] Identified Wide Cam (BSM) via path $WIDE_PATH at index: $WIDE_CAM_IDX"
else
    export WIDE_CAM=0
    export WIDE_CAM_IDX=0
    echo "[WARN] Wide Cam path not found, falling back to default index 0"
fi

# 启动前硬件预检查
wait_for_device() {
    local dev=$1
    local name=$2
    echo "Waiting for $name ($dev)..."
    local timeout=10
    local elapsed=0
    while [ ! -e "$dev" ] && [ $elapsed -lt $timeout ]; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
    if [ -e "$dev" ]; then
        echo "✓ $name found!"
    else
        echo "⚠ Warning: $name ($dev) not found after ${timeout}s"
    fi
}

wait_for_device "/dev/video$ROAD_CAM_IDX" "Road Camera (Long-focus)"
wait_for_device "/dev/video$WIDE_CAM_IDX" "Wide Camera (BSM)"

# 设置动态库路径，使 cam_blindspot 能找到 libonnxruntime.so.1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/third_party/onnxruntime/lib

# 导出环境变量供 cam_blindspot 进程内部使用，确保路径 100% 同步
export WIDE_CAM_DEV="/dev/video$WIDE_CAM_IDX"

# 在后台启动 manager
echo "Starting openpilot manager..."
# 注意：移除 WIDE_CAM=$WIDE_CAM 避免与 cam_blindspot 冲突
SCALE=1 USE_USBCAM=1 ROAD_CAM=$ROAD_CAM NO_DM=1 JY62=0 USE_CAMPATH=1 system/manager/manager.py &
MANAGER_PID=$!

# 等待监控任务完成
# 在后台启动系统管理器
# SCALE=1 USE_WEBCAM=1 USE_CAMPATH=1 python3 system/manager/manager.py &

# 等待UI窗口出现并移动到预设位置 (高频轮询)
echo "Waiting for ui window..."
for i in {1..100}; do
    if wmctrl -l | grep -q "ui"; then
        echo "✓ Found ui window, positioning to (${UI_WINDOW_X}, ${UI_WINDOW_Y})"
        wmctrl -r "ui" -e 0,$UI_WINDOW_X,$UI_WINDOW_Y,$UI_WINDOW_WIDTH,$UI_WINDOW_HEIGHT
        break
    fi
    sleep 0.2
done

# 定义摄像头窗口位置
CAMERA_WINDOW_HEIGHT=528
CAMERA_WINDOW_WIDTH=640

# 定义摄像头窗口预设位置
CAMERA_1_X=737   # Camera 1的X坐标
CAMERA_1_Y=1129  # Camera 1的Y坐标

CAMERA_2_X=2240  # Camera 2的X坐标
CAMERA_2_Y=1129  # Camera 2的Y坐标 (与Camera 1相同)

CAMERA_0_X=1475  # Camera 0的X坐标
CAMERA_0_Y=1129  # Camera 0的Y坐标 (几乎与Camera 1相同)

# 定义窗口排列函数
position_camera_window() {
    local cam_num=$1
    local target_x=$2
    local target_y=$3

    # 等待窗口出现 (高频轮询)
    for i in {1..50}; do
        if wmctrl -l | grep -q "Camera $cam_num"; then
            echo "✓ Moving Camera $cam_num to ($target_x, $target_y)"
            wmctrl -r "Camera $cam_num" -e 0,$target_x,$target_y,$CAMERA_WINDOW_WIDTH,$CAMERA_WINDOW_HEIGHT
            break
        fi
        sleep 0.2
    done
}

# 定位各个摄像头窗口到预设位置
position_camera_window 1 $CAMERA_1_X $CAMERA_1_Y
position_camera_window 2 $CAMERA_2_X $CAMERA_2_Y
position_camera_window 0 $CAMERA_0_X $CAMERA_0_Y

# 最后将UI窗口保持在最前面
sleep 2
if wmctrl -l | grep -q "ui"; then
    wmctrl -a "ui"
fi

# 等待所有后台进程结束
wait