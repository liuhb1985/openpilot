#!/bin/bash
# 重新加载 udev 规则并触发设备扫描
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 666 /dev/bus/usb/*
sleep 1
# 运行op
cd /home/$LOGNAME/openpilot &&
export FINGERPRINT="HYUNDAI_SANTA_FE" &&
export SKIP_FW_QUERY="1" &&
source .venv/bin/activate &&
# 获取当前小时（0~23）
HOUR=$(date +%H)

# 判断时间
if [ "$HOUR" -ge 8 ] && [ "$HOUR" -lt 17 ]; then
    echo "☀️ 白天模式：启用广角 + 长焦摄像头"
    USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py
    #USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py  #使用sensor_jy62
    #USE_WEBCAM=1 ROAD_CAM=0 WIDE_CAM=2 NO_DM=0 JY62=2 system/manager/manager.py  #使用locationd_jy62
else
    echo "🌙 夜间模式：仅启用长焦摄像头"
    #USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py
    #USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py  #使用sensor_jy62
    USE_WEBCAM=1 ROAD_CAM=0 WIDE_CAM=2 NO_DM=0 JY62=2 system/manager/manager.py  #使用locationd_jy62
fi
