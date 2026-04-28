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

# 清理残留进程
pkill -9 camerad 2>/dev/null

# 在后台启动 manager
echo "Starting openpilot manager..."
SCALE=1 NO_DM=1 JY62=0 system/manager/manager.py &

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