#!/bin/bash

# 1. 基础环境配置
export BASEDIR=$(pwd)
export PYTHONPATH=$BASEDIR:$BASEDIR/opendbc_repo:$BASEDIR/panda_repo:$BASEDIR/rednose_repo

# 2. 模拟胜达 (Santa Fe) 的核心参数
export FINGERPRINT="HYUNDAI_SANTA_FE"  # 强制识别为胜达
export SKIP_FW_QUERY=1                 # PC模拟必须跳过硬件查询
export NO_CAN_TIMEOUT=1                # 避免因为回放延迟导致超时报错
export BLOCK_BOOTLOG=1                 # 禁用启动日志上传

# 3. PC 兼容性设置
export USE_WEBCAM=1                    # 使用 USB 摄像头或模拟摄像头
export NO_DM=1                         # 禁用驾驶员监控（PC通常不需要）
export NOSENSOR=1                      # 禁用手机底层传感器检测

# 4. 动态库路径（确保 locationd 和 cam_blindspot 能找到库）
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BASEDIR/third_party/onnxruntime/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BASEDIR/selfdrive/locationd

# 5. 打印提示信息
echo "==========================================="
echo "   CarrotPilot Santa Fe 模拟测试模式启动"
echo "   车型: HYUNDAI_SANTA_FE"
echo "==========================================="

# 6. 启动 manager (主进程管理)
# 使用 & 后台运行，方便在同一个终端显示回放指令
./system/manager/manager.py &
MANAGER_PID=$!

echo ""
echo "系统正在启动... 请在另一个终端运行回放工具："
echo "  ./tools/replay/replay --demo 'cb19803099955e4e|2022-10-14--14-38-04'"
echo ""

# 等待主进程退出
wait $MANAGER_PID
