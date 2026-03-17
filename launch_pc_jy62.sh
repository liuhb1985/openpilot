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
# 获取屏幕尺寸 (manager 启动前先算好)
SCREEN_WIDTH=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr | grep '*' | awk '{print $1}' | head -n1 | cut -d'x' -f2)
SCREEN_WIDTH=${SCREEN_WIDTH:-1920}
SCREEN_HEIGHT=${SCREEN_HEIGHT:-1080}

# 布局设计（左右并排）：
HALF_W=$((SCREEN_WIDTH / 2))

# 终极健壮的窗口定位函数
# 参数: $1=进程名关键字 (用于pgrep)  $2=窗口标题关键字 (备用方案)  $3=x $4=y $5=w $6=h
robust_place() {
    local proc_name="$1"
    local title_key="$2"
    local x=$3 y=$4 w=$5 h=$6
    (
        local wid=""
        # 循环等待进程启动并创建窗口 (最多等 30 秒)
        for i in {1..60}; do
            # 方案 A: 尝试通过进程 PID 查找窗口 ID (最精准)
            local pid=$(pgrep -f "$proc_name" | head -n1)
            if [ -n "$pid" ]; then
                wid=$(xdotool search --pid "$pid" 2>/dev/null | tail -n1)
            fi
            
            # 方案 B: 如果方案 A 没拿到，尝试通过标题模糊匹配 (兼容性)
            if [ -z "$wid" ]; then
                wid=$(xdotool search --name "$title_key" 2>/dev/null | head -n1)
            fi

            if [ -n "$wid" ]; then
                # 方案 C: 如果拿到了 WID，为了对抗 C++ 源码里的 w->move(770, 558) 硬编码，
                # 我们必须在接下来的 10 秒内不断地把窗口“拉回来”。
                echo "[info] Found window for $proc_name (WID: $wid), locking position..."
                for j in {1..20}; do
                    wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz 2>/dev/null
                    xdotool windowmove "$wid" $x $y
                    xdotool windowsize "$wid" $w $h
                    sleep 0.5
                done
                return
            fi
            sleep 0.5
        done
        echo "[warn] Could not find window for $proc_name ($title_key) after 30s"
    ) &
}

# 监控 openpilot UI (进程名通常包含 bin/ui 或 selfdrive/ui)
robust_place "selfdrive/ui/ui" "ui" 0 0 $HALF_W $SCREEN_HEIGHT

# 监控 BSM 摄像头窗口 (进程名 cam_blindspot, 标题 All Cameras)
robust_place "cam_blindspot" "Cameras" $HALF_W 0 $HALF_W $SCREEN_HEIGHT

# 在前台启动 manager
USE_USBCAM=1 ROAD_CAM=2 NO_DM=0 JY62=0 system/manager/manager.py

# 等待监控任务完成
wait