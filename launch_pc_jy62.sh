#!/bin/bash
# 重新加载 udev 规则并触发设备扫描
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/bus/usb/*
# 运行op
cd /data/openpilot && 
source .venv/bin/activate && 
USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 JY62=1 system/manager/manager.py

