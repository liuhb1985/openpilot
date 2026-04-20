#!/bin/bash
# CH347T USB-I2C 驱动一键安装脚本

echo "🔧 CH347T 驱动安装工具"
echo "════════════════════════════════════════════════════════"

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "❌ 请使用 sudo 运行此脚本: sudo bash install_ch347.sh"
    exit 1
fi

# 1. 编译驱动
echo -e "\n1️⃣  编译驱动..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_FILE="$SCRIPT_DIR/third_party/ch347/ch34x_pis.c"
DRIVER_FILE="$SCRIPT_DIR/third_party/ch347/ch34x_pis.ko"

if [ ! -f "$SOURCE_FILE" ]; then
    echo "❌ 源文件不存在: $SOURCE_FILE"
    exit 1
fi

cd "$SCRIPT_DIR/third_party/ch347"
make clean > /dev/null 2>&1
if make; then
    echo "✅ 驱动编译成功"
else
    echo "❌ 驱动编译失败"
    exit 1
fi

# 2. 检查驱动文件是否存在
echo -e "\n2️⃣  检查驱动文件..."
if [ ! -f "$DRIVER_FILE" ]; then
    echo "❌ 驱动文件不存在: $DRIVER_FILE"
    exit 1
fi
echo "✅ 找到驱动文件: $DRIVER_FILE"

# 3. 安装内核模块
echo -e "\n3️⃣  安装内核模块到系统..."
MODULE_DIR="/lib/modules/$(uname -r)/extra"
mkdir -p "$MODULE_DIR"
cp "$DRIVER_FILE" "$MODULE_DIR/"
chmod 644 "$MODULE_DIR/ch34x_pis.ko"
echo "✅ 驱动文件已复制到: $MODULE_DIR"

# 4. 更新模块依赖
echo -e "\n4️⃣  更新模块依赖..."
depmod -a
echo "✅ 模块依赖已更新"

# 5. 配置自动加载
echo -e "\n5️⃣  配置开机自动加载..."
MODULES_LOAD_DIR="/etc/modules-load.d"
mkdir -p "$MODULES_LOAD_DIR"
cat > "$MODULES_LOAD_DIR/ch34x_pis.conf" << EOF
ch34x_pis
EOF
echo "✅ 自动加载配置已写入: $MODULES_LOAD_DIR/ch34x_pis.conf"

# 6. 配置设备权限（udev 规则）
echo -e "\n6️⃣  配置设备权限..."
UDEV_RULES_DIR="/etc/udev/rules.d"
mkdir -p "$UDEV_RULES_DIR"
cat > "$UDEV_RULES_DIR/99-ch347.rules" << EOF
KERNEL=="ch34x_pis*", MODE="0666"
EOF
echo "✅ udev 规则已写入: $UDEV_RULES_DIR/99-ch347.rules"

# 7. 加载驱动
echo -e "\n7️⃣  加载驱动..."
if lsmod | grep -q ch34x_pis; then
    echo "   驱动已加载，先卸载..."
    rmmod ch34x_pis
fi
modprobe ch34x_pis
echo "✅ 驱动已加载"

# 8. 验证安装
echo -e "\n8️⃣  验证安装..."
if lsmod | grep -q ch34x_pis; then
    echo "✅ ch34x_pis 模块已加载"
    lsmod | grep ch34x_pis
else
    echo "❌ ch34x_pis 模块加载失败"
    exit 1
fi

if [ -f "$MODULES_LOAD_DIR/ch34x_pis.conf" ]; then
    echo "✅ 自动加载配置正确"
    cat "$MODULES_LOAD_DIR/ch34x_pis.conf"
else
    echo "❌ 自动加载配置失败"
    exit 1
fi

if [ -f "$UDEV_RULES_DIR/99-ch347.rules" ]; then
    echo "✅ udev 规则配置正确"
    cat "$UDEV_RULES_DIR/99-ch347.rules"
else
    echo "❌ udev 规则配置失败"
    exit 1
fi

echo -e "\n════════════════════════════════════════════════════════"
echo "✅ CH347T 驱动安装完成!"
echo "════════════════════════════════════════════════════════"
echo -e "\n安装内容:"
echo "  ✓ 内核模块: $MODULE_DIR/ch34x_pis.ko"
echo "  ✓ 自动加载: $MODULES_LOAD_DIR/ch34x_pis.conf"
echo "  ✓ 设备权限: $UDEV_RULES_DIR/99-ch347.rules"
echo -e "\n📌 注意事项:"
echo "  - 重启后驱动会自动加载"
echo "  - 如需测试驱动，请连接 CH347T 设备并查看 /dev/ch34x_pis*"
echo "  - 如需重新加载驱动: sudo modprobe -r ch34x_pis && sudo modprobe ch34x_pis"
