#!/bin/bash
# 重新加载 udev 规则并触发设备扫描
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/bus/usb/*

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

# ========================================================
# 方案：通过环境变量直接传递给 cam_blindspot 进程，由其内部调用 cv::moveWindow
# ========================================================
export CAM_X=0
export CAM_Y=$CAM_Y
export CAM_W=$SCREEN_WIDTH
export CAM_H=$CAM_H

# 终极健壮的窗口定位函数 (仅用于 UI，摄像头现在自定位)
robust_place() {
    local key="$1"
    local class_key="$2"
    local x=$3 y=$4 w=$5 h=$6
    (
        local wid=""
        echo "[wait] Searching for window ($key / class: $class_key) ..."

        for i in {1..80}; do
            # 先尝试类名匹配 (最稳)
            if [ -n "$class_key" ]; then
                wid=$(xdotool search --class "$class_key" 2>/dev/null | tail -n1)
            fi

            # 再尝试标题匹配
            if [ -z "$wid" ]; then
                wid=$(xdotool search --name "$key" 2>/dev/null | tail -n1)
            fi

            if [ -n "$wid" ]; then
                echo "[info] OK: Found window '$key' (WID: $wid), stacking..."
                for j in {1..40}; do
                    wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz 2>/dev/null
                    xdotool windowmove "$wid" $x $y
                    xdotool windowsize "$wid" $w $h
                    sleep 0.5
                done
                return
            fi
            sleep 0.5
        done

        echo "[error] TIMEOUT: Could not find '$key' or class '$class_key'"
        echo "Current windows list (wmctrl -l):"
        wmctrl -l
    ) &
}

# 监控 openpilot UI (类名是 ui)
robust_place "ui" "ui" 0 0 $SCREEN_WIDTH $UI_H

# 监控摄像头窗口 (现在已通过 CAM_X/Y 变量由进程内部自定位，脚本无需再找)
# robust_place "Cameras" "Opencv" 0 $CAM_Y $SCREEN_WIDTH $CAM_H

# 在前台启动 manager
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py

# 等待监控任务完成