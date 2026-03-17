#!/bin/bash
# 重新加载 udev 规则并触发设备扫描
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/bus/usb/*
export QT_SCALE_FACTOR=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCREEN_SCALE_FACTORS=
export USE_NATIVE_LANE=1
#sleep 1
# 运行op
#cd /home/$LOGNAME/openpilot &&
cd /home/$LOGNAME/cpv9-temp &&
export FINGERPRINT="HYUNDAI_SANTA_FE" &&
export SKIP_FW_QUERY="1" &&
source .venv/bin/activate &&
#USE_WEBCAM=1 ROAD_CAM=0 WIDE_CAM=2 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
#USE_USBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py  #使用locationd_jy62
# 等待窗口创建
# sleep 1

# 获取屏幕尺寸
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)

# 计算窗口尺寸（底部 1/3 屏幕高度）
WINDOW_HEIGHT=$((SCREEN_HEIGHT / 3))
WINDOW_WIDTH=$((SCREEN_WIDTH / 3))
WINDOW_Y=$((SCREEN_HEIGHT - WINDOW_HEIGHT))

# 定义窗口排列函数
position_camera_window() {
    local cam_num=$1
    local offset_x=$2

    # 等待窗口出现
    for i in {1..20}; do
        if wmctrl -l | grep -q "Camera $cam_num"; then
            # 移动并调整窗口大小
            wmctrl -r "Camera $cam_num" -e 0,$offset_x,$WINDOW_Y,$WINDOW_WIDTH,$WINDOW_HEIGHT
            break
        fi
        sleep 1
    done
}

# 定位各个摄像头窗口
position_camera_window 1 0
position_camera_window 0 $WINDOW_WIDTH
position_camera_window 2 $((WINDOW_WIDTH * 2))

# 等待所有后台进程结束
wait