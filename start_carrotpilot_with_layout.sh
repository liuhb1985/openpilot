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
# 1. 长焦摄像头 (Road Cam)
# 自动检测首个非 metadata 的 video-index0 设备
ROAD_PATH=$(ls /dev/v4l/by-path/*video-index0 2>/dev/null | grep -v "metadata" | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
    export ROAD_CAM=$(basename "$ROAD_PATH")
    echo "[INFO] Identified Road Cam at $ROAD_PATH (Index: $ROAD_CAM_IDX)"
else
    export ROAD_CAM=0
    export ROAD_CAM_IDX=0
    echo "[WARN] Road Cam not found via path, falling back to /dev/video0"
fi

# 2. 广角摄像头 (Wide-angle / BSM, 默认 index 0)
# 注意：video0 的 MJPG 流由 cam_blindspot 独占，webcamerad 不再打开广角摄像头。
# 只记录 BSM 设备索引，供 cam_blindspot 进程使用。
WIDE_PATH=$(ls /dev/v4l/by-path/*usb*-0:1:1.0-video-index0 2>/dev/null | head -n 1)
if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
    WIDE_DEV=$(readlink -f "$WIDE_PATH")
    export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
    echo "[INFO] BSM cam_blindspot uses video${WIDE_CAM_IDX} (webcamerad will NOT open wide camera)"
else
    export WIDE_CAM_IDX=0
    echo "[WARN] Wide Cam path not found, falling back to default index 0"
fi
# 不设置 WIDE_CAM，webcamerad 只负责长焦摄像头
unset WIDE_CAM

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
wait_for_device "/dev/video$WIDE_CAM_IDX" "Wide Camera (BSM, cam_blindspot)"

# 设置动态库路径，使 cam_blindspot 能找到 libonnxruntime.so.1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/third_party/onnxruntime/lib

# 导出环境变量供 cam_blindspot 进程内部使用，确保路径 100% 同步
export WIDE_CAM_DEV="/dev/video$WIDE_CAM_IDX"

# 在后台启动 manager
echo "Starting openpilot manager..."
# 使用 USE_USBCAM=1 启动高效的 C++ 版 camerad (usbcamerad)
# 我们已经在 C++ 代码中修复了路径解析逻辑，现在它能正确处理 $ROAD_CAM (pci-... 字符串)
# 设置 CAM_WIDTH/HEIGHT 以确保以 1080p 模式启动
CAM_WIDTH=1920 CAM_HEIGHT=1080 SCALE=1 USE_USBCAM=1 ROAD_CAM=$ROAD_CAM NO_DM=1 JY62=0 USE_CAMPATH=1 system/manager/manager.py &

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