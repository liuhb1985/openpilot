#!/bin/bash

BASE_DIR="/data"

DIRS=(
    "/data/media"
    "/data/media/0"
    "/data/media/0/realdata"
    "/data/log"
    "/data/stats"
    "/data/tmp"
)

echo "===== OpenPilot PC Fake Android Environment Init ====="

# 1️⃣ 检查 /data
if [ ! -d "$BASE_DIR" ]; then
    echo "Creating $BASE_DIR ..."
    sudo mkdir -p "$BASE_DIR" || { echo "Failed to create $BASE_DIR"; exit 1; }
fi

# 2️⃣ 创建目录结构
for DIR in "${DIRS[@]}"; do
    if [ -d "$DIR" ]; then
        echo "$DIR already exists."
    else
        echo "Creating $DIR ..."
        sudo mkdir -p "$DIR" || { echo "Failed to create $DIR"; exit 1; }
    fi
done

# 3️⃣ 创建 tmux.log
if [ ! -f "/data/media/tmux.log" ]; then
    echo "Creating /data/media/tmux.log ..."
    sudo touch /data/media/tmux.log || { echo "Failed to create tmux.log"; exit 1; }
else
    echo "/data/media/tmux.log already exists."
fi

# 4️⃣ 设置权限（开发环境用）
echo "Setting permissions..."
sudo chmod -R 777 /data

echo "===== Initialization Complete ====="