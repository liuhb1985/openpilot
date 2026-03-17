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
# 先获取屏幕尺寸 (在 manager 启动前计算好)
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)
# 提供缺省值防止 xrandr 失败返回空值导致除法出错
SCREEN_WIDTH=${SCREEN_WIDTH:-1920}
SCREEN_HEIGHT=${SCREEN_HEIGHT:-1080}

# 计算窗口尺寸（底部 1/3 屏幕高度，Camera 0 占中间第三分之一）
WINDOW_HEIGHT=$((SCREEN_HEIGHT / 3))
WINDOW_WIDTH=$((SCREEN_WIDTH / 3))
WINDOW_Y=$((SCREEN_HEIGHT - WINDOW_HEIGHT))

# 关键修复：在 manager 启动之前，以后台并发的方式监听窗口并定位
# 每个摄像头独占一个后台子 shell，每 0.5 秒轮询一次，最长等 20 秒
wait_and_place() {
    local title="$1"
    local x=$2 y=$3 w=$4 h=$5
    for i in {1..40}; do
        if wmctrl -l | grep -q "$title"; then
            wmctrl -r "$title" -e 0,$x,$y,$w,$h
            return
        fi
        sleep 0.5
    done
}

# 目前只配置 Camera 0，放在屏幕底部中间 1/3 区域
wait_and_place "Camera 0" $WINDOW_WIDTH   $WINDOW_Y $WINDOW_WIDTH $WINDOW_HEIGHT &

# 如果后续增加 Camera 1 / Camera 2，取消下面两行注释即可
# wait_and_place "Camera 1" 0              $WINDOW_Y $WINDOW_WIDTH $WINDOW_HEIGHT &
# wait_and_place "Camera 2" $((WINDOW_WIDTH*2)) $WINDOW_Y $WINDOW_WIDTH $WINDOW_HEIGHT &

# 在前台启动 openpilot manager（此时窗口监控已在后台并发运行）
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py

# manager 退出后，等待所有后台窗口监控 shell 也退出
wait