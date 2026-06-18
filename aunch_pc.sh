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

export FINGERPRINT="HONDA_CIVIC_2022"
OP_DIR=$(dirname "$(readlink -f "$0")")
export PARAMS_ROOT="$OP_DIR/params"
export LOG_ROOT="$OP_DIR/.comma"

cd "$OP_DIR" || exit 1
source .venv/bin/activate

# 确保脚本可从仓库根目录导入 cereal/openpilot 包
if [ -n "$PYTHONPATH" ]; then
  export PYTHONPATH="$OP_DIR:$PYTHONPATH"
else
  export PYTHONPATH="$OP_DIR"
fi

FAKE_PANDA_PID=""
cleanup() {
  if [ -n "$FAKE_PANDA_PID" ] && kill -0 "$FAKE_PANDA_PID" 2>/dev/null; then
    echo "停止 fake panda 发布器..."
    kill "$FAKE_PANDA_PID" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# 默认启用 fake panda，适配无实时CAN测试场景
if [ "${ENABLE_FAKE_PANDA:-1}" = "1" ]; then
  echo "启用 fake panda 模式（NOBOARD=1, SKIP_FW_QUERY=1）"
  export NOBOARD=1
  export SKIP_FW_QUERY=1
  export BLOCK="${BLOCK},pandad"
  export FAKE_CAN_PROFILE="${FAKE_CAN_PROFILE:-honda}"
  export FAKE_PANDA_SAFETY_MODEL="${FAKE_PANDA_SAFETY_MODEL:-hondaBosch}"
  python3 "$OP_DIR/tools/sim/fake_panda_state.py" &
  FAKE_PANDA_PID=$!
  sleep 1
fi

USE_USBCAM=1 NO_DM=1 ROAD_CAM=0 WIDE_CAM=2 system/manager/manager.py
# USE_USBCAM=1 NO_DM=1 ROAD_CAM=0 WIDE_CAM=2 system/manager/manager.py
