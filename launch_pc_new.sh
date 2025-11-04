#!/bin/bash
# 一键启动 CarrotPilot PC 版（基于成都日出/日落时间切换）
# ✅ 自动修复 Panda USB 权限 + udev
# ✅ 根据成都时区的日出/日落判断 白天/夜间
# ✅ 白天低速广角/高速长焦，夜间仅长焦

echo "⚡ 开始 CarrotPilot 自动启动脚本…"

# 1️⃣ 修复 Panda USB 权限 & udev 规则
UDEV_FILE="/etc/udev/rules.d/11-panda.rules"
sudo bash -c "cat > $UDEV_FILE" <<'EOF'
# comma.ai Panda USB 权限
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddee", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="f00d", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod -R 666 /dev/bus/usb/
UDEV_DELAY="/etc/udev/rules.d/99-panda-delay.rules"
sudo bash -c "cat > $UDEV_DELAY" <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="bbaa", RUN+="/bin/sleep 1"
EOF
echo "✅ Panda USB 权限与 udev 规则修复完成"

# 等待 Panda 枚举
sleep 1

# 2️⃣ 获取成都的日出/日落时间
LAT="30.67"    # 成都
LNG="104.06"
SUN_TIMES=$(curl -s "https://api.sunrise-sunset.org/json?lat=$LAT&lng=$LNG&formatted=0")
SUNRISE_UTC=$(echo "$SUN_TIMES" | jq -r '.results.sunrise')
SUNSET_UTC=$(echo "$SUN_TIMES"  | jq -r '.results.sunset')

# 转换为成都本地时间（UTC+8）
SUNRISE=$(date -d "$SUNRISE_UTC +8 hours" +"%H:%M")
SUNSET=$(date -d "$SUNSET_UTC +8 hours"  +"%H:%M")
SR_H=$(echo $SUNRISE | cut -d: -f1)
SR_M=$(echo $SUNRISE | cut -d: -f2)
SS_H=$(echo $SUNSET  | cut -d: -f1)
SS_M=$(echo $SUNSET  | cut -d: -f2)

echo "📅 成都日出时间：$SR_H:$SR_M , 日落时间：$SS_H:$SS_M"

# 3️⃣ 激活 virtualenv 并进入 openpilot 目录
cd /home/$LOGNAME/openpilot
source .venv/bin/activate

# 4️⃣ 获取当前成都时间
NOW_H=$(date -d "+8 hour" +%H)
NOW_M=$(date -d "+8 hour" +%M)

# 辅助函数比较时间（小时分钟）
time_less() {
    [ "$1" -lt "$3" ] || ( [ "$1" -eq "$3" ] && [ "$2" -lt "$4" ] )
}

# 判断是否“白天” : 当前时间 >= 日出 && 当前时间 < 日落
if time_less $SR_H $SR_M $NOW_H $NOW_M && time_less $NOW_H $NOW_M $SS_H $SS_M; then
    echo "☀️ 白天模式：启用广角 + 长焦摄像头"
    USE_WEBCAM=1 \
    ROAD_CAM=0 \
    WIDE_CAM=2 \
    NO_DM=0 \
    JY62=1 \
    python3 system/manager/manager.py --auto-speed-cam
else
    echo "🌙 夜间模式：仅启用长焦摄像头"
    USE_WEBCAM=1 \
    ROAD_CAM=0 \
    WIDE_CAM=-1 \
    NO_DM=0 \
    JY62=1 \
    python3 system/manager/manager.py
fi
