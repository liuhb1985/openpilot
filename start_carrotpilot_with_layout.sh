#!/bin/bash

# 设置终端窗口标题为 "Log Window" 以便 wmctrl 精准定位
echo -ne "\033]0;Log Window\007"

# -----------------------------------------------------------------------------
# 系统级性能终极优化 (超低延迟、防卡死、高性能模式)
# -----------------------------------------------------------------------------
echo "正在注入系统级极客优化..."
# 1. 强锁 CPU 高性能模式
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1
# 2. 降低 Swappiness 阻断内存颠簸换出
sudo sysctl -w vm.swappiness=10 >/dev/null 2>&1
# 3. 调大本地套接字内核缓冲区以防 VisionIPC 视频流丢包绿屏
sudo sysctl -w net.core.rmem_max=26214400 >/dev/null 2>&1
sudo sysctl -w net.core.wmem_max=26214400 >/dev/null 2>&1
# 4. 配置 OpenCL / GPU 推理持久化编译缓存
export TI_WITH_OPENCL=1
export PYOPENCL_COMPILER_OUTPUT=0
export SYCL_CACHE_DIR=$(pwd)/.cl_cache
mkdir -p "$SYCL_CACHE_DIR"
echo "系统级优化注入完毕 ✓"
# -----------------------------------------------------------------------------

# 设置 USB 权限
sudo chmod -R 777 /dev/bus/usb/*
sudo chmod 666 /dev/i2c-*
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


# 设置环境变量并激活虚拟环境
export FINGERPRINT="HYUNDAI_SANTA_FE"
export SKIP_FW_QUERY="1"
source .venv/bin/activate

# 等待所有 USB 摄像头加载
check_cameras_ready() {
    echo "Wait for cameras to stabilize..."
    local expected=2
    local timeout=10
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        local count=$(ls /dev/video* 2>/dev/null | wc -l)
        if [ $count -ge $expected ]; then
             echo "✓ Found $count camera devices."
             export CAMERAS_EXIST=1
             return 0
        fi
        sleep 0.2
        elapsed=$((elapsed + 2))
        echo "Searching... ($count/$expected)"
    done

    local final_count=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ $final_count -gt 0 ]; then
        export CAMERAS_EXIST=1
    else
        export CAMERAS_EXIST=0
    fi
    echo "⚠ Warning: Expected $expected cameras but found $final_count. Proceeding anyway..."
}
check_cameras_ready
# -----------------------------------------------------------------------------
# 摄像头持久化路径配置 (解决 errno 25 / 绿屏问题)
# -----------------------------------------------------------------------------
# 1. 长焦摄像头 (Long-focus: RYS RGB Camera)
# 升级为终极的 by-id 物理绑定（永久解决插拔 USB 端口变动问题）
ROAD_PATH=$(ls /dev/v4l/by-id/*RYS_RGB*video-index0 2>/dev/null | head -n 1)
if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
    ROAD_DEV=$(readlink -f "$ROAD_PATH")
    export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
    export ROAD_CAM=$ROAD_CAM_IDX
    echo "[INFO] Identified Road Cam (Long-focus) via ID $ROAD_PATH at index: $ROAD_CAM_IDX"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.4)
    ROAD_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.4:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$ROAD_PATH" ] && [ -L "$ROAD_PATH" ]; then
        ROAD_DEV=$(readlink -f "$ROAD_PATH")
        export ROAD_CAM_IDX=${ROAD_DEV#/dev/video}
        export ROAD_CAM=$ROAD_CAM_IDX
        echo "[INFO] Identified Road Cam (Long-focus) via path $ROAD_PATH at index: $ROAD_CAM_IDX"
    else
        export ROAD_CAM=0
        export ROAD_CAM_IDX=0
        echo "[WARN] Road Cam path not found, falling back to default index 0"
    fi
fi

# 2. 广角摄像头 (Wide-angle: LRCP 500W)
# 升级为终极的 by-id 物理绑定（永久解决插拔 USB 端口变动问题）
WIDE_PATH=$(ls /dev/v4l/by-id/*LRCP_500W*video-index0 2>/dev/null | head -n 1)
if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
    WIDE_DEV=$(readlink -f "$WIDE_PATH")
    export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
    export WIDE_CAM=$WIDE_CAM_IDX
    echo "[INFO] Tele-Camera Enabled: WIDE_PATH via ID $WIDE_PATH at index: $WIDE_CAM"
else
    # 备用方案：采用你提供的新 by-path 路径 (PCI: 65:00.3, USB port 1.2.3)
    WIDE_PATH=$(ls /dev/v4l/by-path/*pci-0000:65:00.3-usb-0:1.2.3:1.0-video-index0 2>/dev/null | head -n 1)
    if [ -n "$WIDE_PATH" ] && [ -L "$WIDE_PATH" ]; then
        WIDE_DEV=$(readlink -f "$WIDE_PATH")
        export WIDE_CAM_IDX=${WIDE_DEV#/dev/video}
        export WIDE_CAM=$WIDE_CAM_IDX
        echo "[INFO] Tele-Camera Enabled: WIDE_PATH via path $WIDE_PATH at index: $WIDE_CAM"
    else
        export WIDE_CAM=2
        export WIDE_CAM_IDX=2
        echo "[WARN] Wide Cam path not found, falling back to default index 2"
    fi
fi

# 启动前硬件预检查
wait_for_device() {
    local dev=$1
    local name=$2

    # 如果初始扫描完全没发现摄像头，或者设备路径未识别，则跳过长时间等待
    if [ "$CAMERAS_EXIST" = "0" ]; then
        echo "[SKIP] $name ($dev) skipped (no cameras detected in initial scan)"
        return 0
    fi

    echo "Waiting for $name ($dev)..."
    local timeout=2  # 缩短单项等待超时
    local elapsed=0
    while [ ! -e "$dev" ] && [ $elapsed -lt $timeout ]; do
        sleep 0.5
        elapsed=$((elapsed + 1))
    done
    if [ -e "$dev" ]; then
        echo "✓ $name found!"
    else
        echo "⚠ Warning: $name ($dev) not found after ${timeout}s"
    fi
}

wait_for_device "/dev/video$ROAD_CAM_IDX" "Road Camera (Long-focus)"
wait_for_device "/dev/video$WIDE_CAM_IDX" "Wide Camera (Wide-angle)"

# 设置动态库路径，使相关进程能找到 libonnxruntime
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/third_party/onnxruntime/lib

# 深度优化：开启 VisionIPC 模式，让盲区监测共享模型摄像头数据，消除硬件占用冲突
export USE_VISION_IPC=1

# 在后台启动 manager
echo "Starting openpilot manager with Dual-Camera Support..."
# 使用完整路径以配合 USE_CAMPATH=1，确保设备识别的唯一性
USE_WEBCAM=1 ROAD_CAM=$ROAD_PATH WIDE_CAM=$WIDE_PATH NO_DM=1 IMU_SEL=1 USE_CAMPATH=0 system/manager/manager.py &

MANAGER_PID=$!

# 等待监控任务完成
# 在后台启动系统管理器
# SCALE=1 USE_WEBCAM=1 USE_CAMPATH=1 python3 system/manager/manager.py &

# -----------------------------------------------------------------------------
# 窗口布局参数定义 (适配 2048x1536 高清屏幕 - 黄金分割对称分布)
# -----------------------------------------------------------------------------
# 1. 主界面窗口 (恢复原来默认尺寸 2048x1080)
UI_WINDOW_X=0
UI_WINDOW_Y=0
UI_WINDOW_WIDTH=2048
UI_WINDOW_HEIGHT=1080

# 2. 盲区画面窗口 (调整为标准的 640x360，完美贴合主界面底部，不大不小正好)
CAMERA_WINDOW_WIDTH=640
CAMERA_WINDOW_HEIGHT=360
CAMERA_0_X=0
CAMERA_0_Y=1080
CAMERA_1_X=0
CAMERA_1_Y=1080
CAMERA_2_X=0
CAMERA_2_Y=1080

# 3. 日志命令窗口 (自适应拓宽，占据盲区窗口往右的所有剩余空间，尺寸 1408x456)
LOG_WINDOW_X=640
LOG_WINDOW_Y=1080
LOG_WINDOW_WIDTH=1408
LOG_WINDOW_HEIGHT=456

# -----------------------------------------------------------------------------
# 终极自适应窗口布局分配守护进程 (高频高可靠持续对齐拼接，打败延迟启动与抢占)
# -----------------------------------------------------------------------------
maintain_layout() {
    echo "Starting layout guardian daemon..."
    local elapsed=0
    local term_win_id=""

    # 1. 尝试寻找当前运行该脚本终端的 Window ID
    if [ ! -z "$WINDOWID" ]; then
        term_win_id="$WINDOWID"
    else
        # 沿着当前进程树向上搜索，直到找到归属于当前进程族终端窗口的 ID
        local pid=$$
        while [ "$pid" -gt 1 ]; do
            term_win_id=$(wmctrl -lp 2>/dev/null | grep " $pid " | head -n 1 | cut -d' ' -f1)
            if [ ! -z "$term_win_id" ]; then
                break
            fi
            pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
            [ -z "$pid" ] && break
        done
    fi

    if [ ! -z "$term_win_id" ]; then
        echo "✓ Term Window found via process tree tracing: $term_win_id"
    else
        echo "⚠ Term Window not found via process tree tracing. Falling back to title matching..."
    fi

    local aligned_ui=""
    local aligned_cam=""
    local aligned_term=""

    # 持续维护 30 秒布局，以 0.5 秒为间隔
    while [ $elapsed -lt 60 ]; do
        # 1. 布局 UI 窗口
        local ui_win=$(wmctrl -l | grep -Ei "openpilot|MainWindow|\bui\b|^ui$" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
        if [ ! -z "$ui_win" ] && [ "$ui_win" != "$aligned_ui" ]; then
            wmctrl -i -r "$ui_win" -b remove,fullscreen,maximized_horz,maximized_vert
            wmctrl -i -r "$ui_win" -b remove,decorations
            if command -v xprop >/dev/null 2>&1; then
                xprop -id "$ui_win" -f _MOTIF_WM_HINTS 32c -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" 2>/dev/null
            fi
            wmctrl -i -r "$ui_win" -e 0,$UI_WINDOW_X,$UI_WINDOW_Y,$UI_WINDOW_WIDTH,$UI_WINDOW_HEIGHT
            aligned_ui="$ui_win"
        fi

        # 核心智能对齐算法：动态检测主 UI 窗口在桌面环境中的实际底部 Y 坐标，作为下方盲区和日志窗口的起始顶部坐标 Y
        local target_y=1080
        if [ ! -z "$ui_win" ]; then
            local ui_geom=$(wmctrl -l -G | grep -i "$ui_win" | head -n 1)
            # wmctrl -l -G 输出中，第 4 列是实际 Y，第 6 列是实际 Height
            local actual_y=$(echo "$ui_geom" | awk '{print $4}')
            local actual_h=$(echo "$ui_geom" | awk '{print $6}')
            # 验证实际值是否是合法正整数，避免异常环境运算报错
            if [ ! -z "$actual_y" ] && [ ! -z "$actual_h" ] && [ "$actual_y" -eq "$actual_y" ] 2>/dev/null; then
                target_y=$((actual_y + actual_h))
            fi
        fi

        # 2. 布局 All Cameras 盲区检测窗口 (兼容单/多窗口模式，顶部使用动态计算的 target_y 进行完美对齐)
        local cam_win=$(wmctrl -l | grep -Ei "All Cameras|camera|cameras|Camera 0" | grep -vEi "code|chrome|firefox" | head -n 1 | cut -d' ' -f1)
        if [ ! -z "$cam_win" ] && [ "$cam_win" != "$aligned_cam" ]; then
            wmctrl -i -r "$cam_win" -b remove,fullscreen,maximized_horz,maximized_vert
            wmctrl -i -r "$cam_win" -b remove,decorations
            if command -v xprop >/dev/null 2>&1; then
                xprop -id "$cam_win" -f _MOTIF_WM_HINTS 32c -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" 2>/dev/null
            fi
            wmctrl -i -r "$cam_win" -e 0,$CAMERA_0_X,$target_y,$CAMERA_WINDOW_WIDTH,$CAMERA_WINDOW_HEIGHT
            aligned_cam="$cam_win"
        fi

        # 3. 布局终端日志窗口 (优先精准寻找当前脚本控制的 "Log Window"，顶部使用 target_y 与盲区窗口绝对对齐)
        local active_term_id="$term_win_id"
        if [ -z "$active_term_id" ]; then
            # 优先精准匹配标题包含 "Log Window"
            active_term_id=$(wmctrl -l | grep -Ei "Log Window" | head -n 1 | cut -d' ' -f1)
        fi
        if [ -z "$active_term_id" ]; then
            # 备选模糊标题匹配
            active_term_id=$(wmctrl -l | grep -Ei "terminal|bash" | head -n 1 | cut -d' ' -f1)
        fi
        if [ ! -z "$active_term_id" ] && [ "$active_term_id" != "$aligned_term" ]; then
            wmctrl -i -r "$active_term_id" -b remove,fullscreen,maximized_horz,maximized_vert
            wmctrl -i -r "$active_term_id" -b remove,decorations
            if command -v xprop >/dev/null 2>&1; then
                xprop -id "$active_term_id" -f _MOTIF_WM_HINTS 32c -set _MOTIF_WM_HINTS "0x2, 0x0, 0x0, 0x0, 0x0" 2>/dev/null
            fi
            wmctrl -i -r "$active_term_id" -e 0,$LOG_WINDOW_X,$target_y,$LOG_WINDOW_WIDTH,$LOG_WINDOW_HEIGHT
            aligned_term="$active_term_id"
        fi

        # 如果三个窗口均已对齐，则提前优雅退出守护进程
        if [ ! -z "$aligned_ui" ] && [ ! -z "$aligned_cam" ] && [ ! -z "$aligned_term" ]; then
            echo "✓ All windows successfully aligned. Layout daemon exiting early."
            break
        fi

        sleep 0.5
        elapsed=$((elapsed + 1))
    done

    echo "Layout guardian daemon finished."
}

# 后台异步启动守护进程
maintain_layout &

# 等待所有后台进程结束
wait