#!/bin/bash
sudo chmod -R 777 /dev/bus/usb/* 
sudo udevadm control --reload-rules && sudo udevadm trigger
# 等待CH347 I2C总线就绪（最多30秒）
echo "等待 CH347 I2C 总线..."
for i in {1..30}; do
    if grep -l "ch347" /sys/bus/i2c/devices/i2c-*/name 2>/dev/null; then
        echo "CH347 I2C 总线就绪 ✓"
        sleep 1
        break
    fi
    echo "等待中... ($i/30)"
    sleep 1
done

# 确保 CH347 I2C 设备权限正确
CH347_BUS=$(grep -l "ch347" /sys/bus/i2c/devices/i2c-*/name 2>/dev/null | head -1 | sed 's|.*/i2c-\([0-9]*\)/.*|\1|')
if [ -n "$CH347_BUS" ] && [ -e "/dev/i2c-$CH347_BUS" ]; then
    sudo chmod 666 "/dev/i2c-$CH347_BUS" 2>/dev/null
fi

# 最终检查
if ! grep -q "ch347" /sys/bus/i2c/devices/i2c-*/name 2>/dev/null; then
    echo "❌ CH347 I2C 总线未就绪，退出"
    exit 1
fi

export FINGERPRINT="BYD HAN EV 20"
OP_DIR=$(dirname "$(readlink -f "$0")")
export PYTHONPATH="$OP_DIR:$PYTHONPATH"
export PARAMS_ROOT="$OP_DIR/params"
export LOG_ROOT="$OP_DIR/.comma"
cd "$OP_DIR" &&
source .venv/bin/activate &&
USE_WEBCAM=1 NO_DM=1 ROAD_CAM=0 system/manager/manager.py
