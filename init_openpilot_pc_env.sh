#!/bin/bash

BASE_DIR="/data"

DIRS=(
    # ---- 核心运行目录 (hw.py / hardwared.py) ----
    "/data/media"
    "/data/media/0"
    "/data/media/0/realdata"      # 行车日志存储 (log_root)
    "/data/log"                   # swaglog 日志 (swaglog_root)
    "/data/stats"                 # 统计数据文件 (stats_root, statsd.py)
    "/data/params"                # 参数存储 (hw.h)
    "/data/params/d"              # 参数数据目录 (installer.cc, apilot.py)

    # ---- 媒体与录像 (screenrecorder / carrot_server / frogpilot) ----
    "/data/media/0/videos"        # 屏幕录像 (screenrecorder.cc)

    # ---- 构建与更新 (build.py / updated.py) ----
    "/data/scons_cache"           # SCons 编译缓存 (build.py)
    "/data/safe_staging"          # OTA 更新暂存 (updated.py)

    # ---- 系统配置 (carrot_serv.py) ----
    "/data/etc"                   # 时区等系统配置 (localtime)

    # ---- 社区功能 (frogpilot) ----
    "/data/community"
    "/data/community/crashes"     # 崩溃日志 (fleetmanager)

    # ---- 临时目录 ----
    "/data/tmp"                   # 临时文件
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
        echo "  [OK] $DIR"
    else
        echo "  [NEW] $DIR"
        sudo mkdir -p "$DIR" || { echo "Failed to create $DIR"; exit 1; }
    fi
done

# 3️⃣ 创建必要的文件
if [ ! -f "/data/media/tmux.log" ]; then
    echo "  [NEW] /data/media/tmux.log"
    sudo touch /data/media/tmux.log || { echo "Failed to create tmux.log"; exit 1; }
else
    echo "  [OK] /data/media/tmux.log"
fi

# 4️⃣ 设置权限（开发环境用）
echo "Setting permissions..."
sudo chmod -R 777 /data

echo "===== Initialization Complete ====="