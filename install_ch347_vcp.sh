#!/bin/bash
# CH347T USB-I2C 驱动安装脚本（AYSTARIK 内核 I2C 子系统版）
# 功能：编译 mfd-ch347.ko + i2c-ch347.ko，替代旧的 ch34x_pis + libch347.so
#
# 相比旧驱动的变化：
#   - 走 Linux 内核 I2C 子系统，标准 /dev/i2c-* 设备
#   - 请求-回复绑定提交（ch347_data_xfer 原子操作）
#   - 首次上电有 fixup 处理脏数据
#   - 可用 i2cdetect/i2cget 等标准工具调试

echo "CH347T 驱动安装工具（AYSTARIK 内核 I2C 版）"
echo "============================================="

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行: sudo bash install_ch347_vcp.sh"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/third_party/ch347_vcp"
KERNEL_VERSION=$(uname -r)
MODULE_DIR="/lib/modules/$KERNEL_VERSION/extra"

# ── 步骤0：清理旧驱动（ch34x_pis + libch347.so）──────────────
echo -e "\n[0/5] 清理旧版 ch34x_pis 驱动..."

# 卸载内核模块
if lsmod | grep -q ch34x_pis; then
    rmmod ch34x_pis 2>/dev/null && echo "  已卸载 ch34x_pis 模块" || echo "  警告: ch34x_pis 卸载失败（可能正在使用）"
fi

# 删除旧的 .ko 文件
find /lib/modules/ -name "ch34x_pis.ko" -delete 2>/dev/null && echo "  已删除旧 ch34x_pis.ko"

# 删除旧的 udev 规则
rm -f /etc/udev/rules.d/99-ch347.rules && echo "  已删除旧 udev 规则"

# 删除 libch347.so
rm -f /usr/local/lib/libch347.so
ldconfig 2>/dev/null && echo "  已删除 libch347.so"

# 清理旧版 AYSTARIK 模块（如果之前装过）
for mod in i2c-ch347 gpio-ch347 spi-ch347 mfd-ch347; do
    if lsmod | grep -q "$mod"; then
        rmmod "$mod" 2>/dev/null
    fi
done
find /lib/modules/ -name "mfd-ch347.ko" -delete 2>/dev/null
find /lib/modules/ -name "i2c-ch347.ko" -delete 2>/dev/null
find /lib/modules/ -name "gpio-ch347.ko" -delete 2>/dev/null
find /lib/modules/ -name "spi-ch347.ko" -delete 2>/dev/null

# 清理其他旧版遗留
rm -f /etc/systemd/system/ch347-load.service
rm -f /etc/systemd/system/ch347-check.service
rm -f /etc/modprobe.d/ch347.conf
rm -f /usr/local/bin/ch347-force-load.sh
rm -f /usr/local/bin/ch347-device-check.sh
rm -f /var/log/ch347-check.log
systemctl daemon-reload 2>/dev/null || true

echo "  旧版驱动清理完成"

# ── 步骤1：根据当前内核编译驱动 ───────────────────────────
echo -e "\n[1/5] 编译 AYSTARIK 驱动模块（目标内核: $KERNEL_VERSION）..."

# 检查并安装内核头文件
KERNEL_HEADERS="/lib/modules/$KERNEL_VERSION/build"
if [ ! -d "$KERNEL_HEADERS" ]; then
    echo "  未找到内核头文件，尝试安装 linux-headers-$KERNEL_VERSION ..."
    if command -v apt-get &>/dev/null; then
        apt-get install -y "linux-headers-$KERNEL_VERSION"
    elif command -v dnf &>/dev/null; then
        dnf install -y "kernel-devel-$KERNEL_VERSION"
    fi
fi

if [ ! -d "$KERNEL_HEADERS" ]; then
    echo "  错误：无法获取内核头文件，请先安装: sudo apt-get install linux-headers-$(uname -r)"
    exit 1
fi

# 检查编译工具链
if ! command -v make &>/dev/null; then
    echo "  安装编译工具..."
    apt-get install -y build-essential 2>/dev/null || true
fi

# 检查源码是否存在
if [ ! -f "$SRC_DIR/mfd-ch347.c" ] || [ ! -f "$SRC_DIR/i2c-ch347.c" ]; then
    echo "  错误：源码文件缺失 ($SRC_DIR)"
    exit 1
fi

echo "  开始编译（只编译 mfd-ch347 + i2c-ch347）..."
BUILD_TMP=$(mktemp -d)

# 只复制需要的文件
cp "$SRC_DIR/mfd-ch347.c" "$BUILD_TMP/"
cp "$SRC_DIR/i2c-ch347.c" "$BUILD_TMP/"
cp "$SRC_DIR/ch347.h"     "$BUILD_TMP/"

# 创建只编译 mfd + i2c 的 Makefile
cat > "$BUILD_TMP/Makefile" << 'MAKEFILE_EOF'
PWD := $(shell pwd)
KVERSION := $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build

obj-m += mfd-ch347.o
obj-m += i2c-ch347.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
MAKEFILE_EOF

if ! make -C "$KERNEL_HEADERS" M="$BUILD_TMP" modules 2>&1; then
    rm -rf "$BUILD_TMP"
    echo "  编译失败！请检查内核头文件是否与当前内核匹配"
    exit 1
fi

if [ ! -f "$BUILD_TMP/mfd-ch347.ko" ] || [ ! -f "$BUILD_TMP/i2c-ch347.ko" ]; then
    rm -rf "$BUILD_TMP"
    echo "  编译完成但未生成所有 .ko 文件"
    exit 1
fi

echo "  编译成功!"

# ── 步骤2：安装到系统 ─────────────────────────────────────
echo -e "\n[2/5] 安装内核模块..."
mkdir -p "$MODULE_DIR"
cp "$BUILD_TMP/mfd-ch347.ko" "$MODULE_DIR/"
cp "$BUILD_TMP/i2c-ch347.ko" "$MODULE_DIR/"
chmod 644 "$MODULE_DIR/mfd-ch347.ko"
chmod 644 "$MODULE_DIR/i2c-ch347.ko"
rm -rf "$BUILD_TMP"
depmod -a
echo "  已安装到: $MODULE_DIR/mfd-ch347.ko"
echo "  已安装到: $MODULE_DIR/i2c-ch347.ko"

# ── 步骤3：配置 udev 规则 ────────────────────────────────
echo -e "\n[3/5] 配置 udev 规则..."
cat > "/etc/udev/rules.d/99-ch347_vcp.rules" << 'EOF'
# CH347T USB-I2C (VID=1a86 PID=55db Mode1, PID=55dd Mode3)
# 设备权限
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55db", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55dd", MODE="0666"
# I2C 设备节点权限（AYSTARIK 驱动创建标准 /dev/i2c-* 设备）
KERNEL=="i2c-[0-9]*", MODE="0666", GROUP="plugdev"
# 设备插入时自动加载驱动
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55db", RUN+="/sbin/modprobe mfd-ch347"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55dd", RUN+="/sbin/modprobe mfd-ch347"
EOF

udevadm control --reload-rules
udevadm trigger
echo "  udev规则已配置（插拔设备时自动加载驱动）"

# ── 步骤4：立即加载驱动 ──────────────────────────────────
echo -e "\n[4/5] 加载驱动模块..."

# 先加载 MFD 核心，再加载 I2C
if modprobe mfd-ch347 2>&1; then
    echo "  mfd-ch347 加载成功"
else
    echo "  警告: mfd-ch347 加载失败（可能设备未插入）"
fi

sleep 1

if modprobe i2c-ch347 2>&1; then
    echo "  i2c-ch347 加载成功"
else
    echo "  警告: i2c-ch347 加载失败"
fi

sleep 1

# ── 步骤5：检测 I2C 总线 ─────────────────────────────────
echo -e "\n[5/5] 检测 CH347 I2C 总线..."

# 查找 CH347 创建的 I2C 总线
CH347_BUS=""
if [ -d /sys/bus/i2c ]; then
    for bus_dir in /sys/bus/i2c/devices/i2c-*; do
        if [ -d "$bus_dir" ]; then
            bus_name=$(cat "$bus_dir/name" 2>/dev/null || echo "")
            if echo "$bus_name" | grep -qi "ch347"; then
                bus_num=$(basename "$bus_dir" | sed 's/i2c-//')
                CH347_BUS=$bus_num
                echo "  找到 CH347 I2C 总线: /dev/i2c-$bus_num (name=$bus_name)"
            fi
        fi
    done
fi

if [ -z "$CH347_BUS" ]; then
    echo "  未检测到 CH347 I2C 总线"
    echo "  可能原因：设备未插入，或驱动加载失败"
    echo "  插入设备后可运行以下命令检查："
    echo "    lsmod | grep ch347"
    echo "    ls /dev/i2c-*"
    echo "    for d in /sys/bus/i2c/devices/i2c-*/; do echo \$(basename \$d): \$(cat \$d/name 2>/dev/null); done"
else
    # 安装 i2c-tools 以便调试
    if ! command -v i2cdetect &>/dev/null; then
        echo "  安装 i2c-tools..."
        apt-get install -y i2c-tools 2>/dev/null || true
    fi
    if command -v i2cdetect &>/dev/null; then
        echo -e "\n  I2C 总线扫描结果:"
        i2cdetect -y "$CH347_BUS" 2>&1 | head -20
    fi
fi

# ── 完成 ───────────────────────────────────────────────────
echo ""
echo "============================================="
echo "安装完成!"
echo "============================================="
echo ""
echo "已安装组件:"
echo "  内核模块: $MODULE_DIR/mfd-ch347.ko"
echo "  内核模块: $MODULE_DIR/i2c-ch347.ko"
echo "  udev规则: /etc/udev/rules.d/99-ch347_vcp.rules"
echo ""
echo "已移除旧组件:"
echo "  ch34x_pis.ko (旧内核模块)"
echo "  libch347.so (旧用户空间库)"
echo "  99-ch347.rules (旧udev规则)"
echo ""
if [ -n "$CH347_BUS" ]; then
    echo "CH347 I2C 总线: /dev/i2c-$CH347_BUS"
    echo ""
fi
echo "调试命令:"
echo "  lsmod | grep ch347          # 查看模块加载状态"
echo "  i2cdetect -y <bus>          # 扫描 I2C 设备"
echo "  i2cget -y <bus> 0x6B 0x0F  # 读 WHO_AM_I"
echo ""
echo "内核升级后重新运行此脚本即可重新编译:"
echo "  sudo bash $0"
