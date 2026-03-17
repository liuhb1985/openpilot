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
cd /home/$LOGNAME/openpilot &&
export FINGERPRINT="HYUNDAI_SANTA_FE" &&
export SKIP_FW_QUERY="1" &&
source .venv/bin/activate &&
#USE_WEBCAM=1 ROAD_CAM=0 WIDE_CAM=2 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
#USE_USBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py  #使用locationd_jy62
# 获取屏幕尺寸 (manager 启动前先算好)
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)
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
UI_H=$((SCREEN_HEIGHT * 67 / 100))
CAM_H=$((SCREEN_HEIGHT - UI_H))
CAM_Y=$UI_H

# 终极健壮的窗口定位函数
robust_place() {
    local key="$1"
    local x=$2 y=$3 w=$4 h=$5
    (
        local wid=""
        echo "[wait] Searching for window: $key ..."

        # 循环等待进程启动并创建窗口 (最多等 40 秒)
        for i in {1..80}; do
            # 方案 A: 尝试通过关键词搜索标题 (不区分大小写)
            wid=$(xdotool search --name --onlyvisible "$key" 2>/dev/null | tail -n1)

            # 方案 B: 如果还没找到，尝试搜索窗口类名
            if [ -z "$wid" ]; then
                wid=$(xdotool search --class --onlyvisible "$key" 2>/dev/null | tail -n1)
            fi

            if [ -n "$wid" ]; then
                echo "[info] OK: Found window '$key' (WID: $wid), positioning..."
                # 持续强力重置位置 15 秒，确保稳定
                for j in {1..30}; do
                    wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz 2>/dev/null
                    xdotool windowmove "$wid" $x $y
                    xdotool windowsize "$wid" $w $h
                    sleep 0.5
                done
                return
            fi
            sleep 0.5
        done
        echo "[error] TIMEOUT: Could not find window containing '$key'"
        wmctrl -l
    ) &
}

# 监控 openpilot UI (铺满上方 2/3)
robust_place "ui" 0 0 $SCREEN_WIDTH $UI_H

# 监控摄像头窗口 (占下方 1/3)
robust_place "Camera" 0 $CAM_Y $SCREEN_WIDTH $CAM_H

# 在前台启动 manager
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py

# 等待监控任务完成
wait