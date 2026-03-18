#!/bin/bash

# 1. 设置环境变量
export PYTHONPATH=$PWD:$PYTHONPATH

# 相机配置 (根据您的硬件微调)
export WEBCAM_W=1920
export WEBCAM_H=1080
export WEBCAM_FOCAL=2520        # 标准/长焦相机焦距
export WEBCAM_WIDE_FOCAL=1150   # 广角相机焦距
export V4L2_DEVICE="/dev/video0"       # 标准/长焦摄像头设备
export V4L2_WIDE_DEVICE="/dev/video1"  # 广角摄像头设备

# 性能优化配置
export USE_THNEED=1          # 启用 Tinygrad 优化
export USE_WEBCAM=1          # 强制使用 USB 摄像头模式
export GRAPHICS_DEVICE=0     # 指定 GPU 设备索引 (如有多个 GPU)

# 2. 启动前置服务 (可选，如果 manager 没启动它们)
# 如果您想单独测试摄像头画面，可以先运行: ./system/camerad/camerad &

# 3. 运行 openpilot 管理器
echo "正在启动 openpilot OPKR PC 版..."
python3 selfdrive/manager/manager.py
