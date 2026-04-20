#!/bin/bash

# 设置 USB 权限
sudo chmod -R 777 /dev/bus/usb/*

# 进入 MinerU 目录

# 设置环境变量并激活虚拟环境
export FINGERPRINT="HYUNDAI_SANTA_FE"
export SKIP_FW_QUERY="1"
source .venv/bin/activate

# 等待所有USB摄像头加载完成
wait_for_cameras() {
    echo "Waiting for USB cameras to be loaded..."

    # 设置超时时间（秒）
    local timeout=120
    local elapsed=0
    local check_interval=2

    # 期望的摄像头数量
    local expected_cameras=4

    # 获取当前已检测到的摄像头数量
    local last_camera_count=0
    local stable_count=0
    local stable_threshold=3  # 连续3次检测到相同数量摄像头才认为稳定

    while [ $elapsed -lt $timeout ]; do
        # 检测当前连接的摄像头数量（通过v4l2设备）
        local current_camera_count=$(ls /dev/video* 2>/dev/null | wc -l)

        if [ $current_camera_count -gt 0 ]; then
            echo "Found $current_camera_count camera device(s) (expected: $expected_cameras)"

            # 检查摄像头数量是否稳定
            if [ $current_camera_count -eq $last_camera_count ]; then
                stable_count=$((stable_count + 1))
                echo "Camera count stable for $stable_count checks"

                if [ $stable_count -ge $stable_threshold ]; then
                    if [ $current_camera_count -eq $expected_cameras ]; then
                        echo "✓ All $expected_cameras cameras loaded successfully"
                        echo "Waiting additional 2 seconds for final initialization..."
                        sleep 2
                        return 0
                    else
                        echo "⚠ Warning: Only $current_camera_count cameras found (expected $expected_cameras)"
                        echo "Proceeding with available cameras..."
                        sleep 2
                        return 0
                    fi
                fi
            else
                stable_count=0
                last_camera_count=$current_camera_count
            fi
        else
            echo "No cameras found yet..."
            last_camera_count=0
            stable_count=0
        fi

        sleep $check_interval
        elapsed=$((elapsed + check_interval))
        echo "Elapsed: ${elapsed}s / ${timeout}s"
    done

    # 超时处理
    echo "⚠ Warning: Timed out waiting for cameras to stabilize"
    echo "Current camera count: $current_camera_count (expected: $expected_cameras)"
    echo "Proceeding anyway, some cameras may not be available..."

    return 0
}

# 执行摄像头等待
wait_for_cameras
# -----------------------------------------------------------------------------
# 摄像头持久化路径配置 (解决 errno 25 / 绿屏问题)
# -----------------------------------------------------------------------------
# 1. 长焦摄像头 (Long-focus, 默认 index 2)
# 使用通配符匹配，解决 usb-0 vs usbv2-0 或微小路径变动
ROAD_PATH=$(ls /dev/v4l/by-path/*usb*-0:2.3:1.0-video-index0 2>/dev/null | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM=${ROAD_DEV#/dev/video}
    echo "[INFO] Identified Road Cam (Long-focus) via path $ROAD_PATH at index: $ROAD_CAM"
else
    export ROAD_CAM=2
    echo "[WARN] Road Cam path not found, falling back to default index 2"
fi

# 2. 广角摄像头 (Wide-angle / BSM, 默认 index 0)
WIDE_PATH=$(ls /dev/v4l/by-path/*usb*-0:1:1.0-video-index0 2>/dev/null | head -n 1)
if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
    WIDE_DEV=$(readlink -f "$WIDE_PATH")
    export WIDE_CAM=${WIDE_DEV#/dev/video}
    echo "[INFO] Identified Wide Cam (BSM) via path $WIDE_PATH at index: $WIDE_CAM"
else
    export WIDE_CAM=0
    echo "[WARN] Wide Cam path not found, falling back to default index 0"
fi

# 启动前硬件预检查
wait_for_device "/dev/video$ROAD_CAM" "Road Camera (Long-focus)"
wait_for_device "/dev/video$WIDE_CAM" "Wide Camera (BSM)"

# 导出环境变量供 cam_blindspot 进程内部使用，确保路径 100% 同步
export WIDE_CAM_DEV="/dev/video$WIDE_CAM"

# 在前台启动 manager (开启精简模式)
SCALE=1 USE_USBCAM=1 ROAD_CAM=$ROAD_CAM WIDE_CAM=$WIDE_CAM NO_DM=0 JY62=0 USE_CAMPATH=1 system/manager/manager.py

# 等待监控任务完成
# 在后台启动系统管理器
# SCALE=1 USE_WEBCAM=1 USE_CAMPATH=1 python3 system/manager/manager.py &

# 等待窗口创建
sleep 8

# 获取屏幕尺寸
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)

# 输出调试信息
echo "Screen resolution: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"

# 定义预设位置 - 根据当前实际窗口位置
UI_WINDOW_X=720      # UI窗口的X坐标 (从当前位置获取)
UI_WINDOW_Y=60      # UI窗口的Y坐标 (从当前位置获取)
UI_WINDOW_WIDTH=2160 # UI窗口的宽度
UI_WINDOW_HEIGHT=1080 # UI窗口的高度

# 等待UI窗口出现并移动到预设位置
for i in {1..30}; do
    if wmctrl -l | grep -q "ui"; then
        echo "Found ui window, moving to (${UI_WINDOW_X}, ${UI_WINDOW_Y})"
        wmctrl -r "ui" -e 0,$UI_WINDOW_X,$UI_WINDOW_Y,$UI_WINDOW_WIDTH,$UI_WINDOW_HEIGHT
        break
    else
        echo "Waiting for ui window... ($i/30)"
    fi
    sleep 1
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

    # 等待窗口出现
    for i in {1..30}; do
        if wmctrl -l | grep -q "Camera $cam_num"; then
            echo "Moving Camera $cam_num to ($target_x, $target_y)"
            wmctrl -r "Camera $cam_num" -e 0,$target_x,$target_y,$CAMERA_WINDOW_WIDTH,$CAMERA_WINDOW_HEIGHT
            break
        else
            echo "Waiting for Camera $cam_num... ($i/30)"
        fi
        sleep 1
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