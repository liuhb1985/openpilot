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
# 先获取屏幕尺寸 (manager 启动前先算好)
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)
SCREEN_WIDTH=${SCREEN_WIDTH:-1920}
SCREEN_HEIGHT=${SCREEN_HEIGHT:-1080}

# 布局设计：
#  ┌──────────────────────────────┐
#  │                              │  ← openpilot UI (高度占屏幕 2/3)
#  │                              │
#  ├──────────────────────────────┤
#  │       Camera 0 (road)        │  ← 摄像头预览 (高度占屏幕 1/3)
#  └──────────────────────────────┘

CAM_H=$((SCREEN_HEIGHT / 3))
UI_H=$((SCREEN_HEIGHT - CAM_H))
CAM_Y=$UI_H

# 通用的窗口定位函数 (无感地在后台轮询)
wait_and_place() {
    local title="$1"
    local x=$2 y=$3 w=$4 h=$5
    (
        for i in {1..60}; do
            if wmctrl -l | grep -q "$title"; then
                wmctrl -r "$title" -e 0,$x,$y,$w,$h
                return
            fi
            sleep 0.5
        done
        echo "[warn] window '$title' not found after 30s"
    ) &
}

# 启动所有窗口监控——必须在 manager 启动之前，否则它们永远不会执行
# openpilot UI → 铺满上方 2/3
wait_and_place "ui" 0 0 $SCREEN_WIDTH $UI_H
# Camera 0 路面摄像头预览 → 占下方 1/3
wait_and_place "Camera 0" 0 $CAM_Y $SCREEN_WIDTH $CAM_H

# 在前台启动 manager（此时上方的窗口监控已经在后台并发运行）
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py

# manager 退出后等待后台子 shell 结束
wait