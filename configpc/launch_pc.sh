#!/bin/bash

# start_carrotpilot.sh
# 用于启动 CarrotPilot 的脚本

# 设置 openpilot 目录路径（根据你的实际路径修改）
OPENPILOT_DIR="$HOME/openpilot"

# 检查目录是否存在
if [ ! -d "$OPENPILOT_DIR" ]; then
    echo "错误: openpilot 目录不存在: $OPENPILOT_DIR"
    exit 1
fi

# 进入目录
cd "$OPENPILOT_DIR" || exit 1

# 检查脚本是否存在
if [ ! -f "start_carrotpilot_with_layout.sh" ]; then
    echo "错误: start_carrotpilot_with_layout.sh 不存在于当前目录"
    echo "当前目录: $(pwd)"
    ls -la | grep -E "start.*carrotpilot"
    exit 1
fi

# 给脚本添加执行权限（如果需要）
if [ ! -x "start_carrotpilot_with_layout.sh" ]; then
    echo "添加执行权限..."
    chmod +x start_carrotpilot_with_layout.sh
fi

# 执行脚本
echo "正在启动 CarrotPilot..."
./start_carrotpilot_with_layout.sh

# 如果上面的命令失败，尝试用 bash 执行
if [ $? -ne 0 ]; then
    echo "直接执行失败，尝试使用 bash 运行..."
    bash start_carrotpilot_with_layout.sh
fi
