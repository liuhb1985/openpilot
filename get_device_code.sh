#!/bin/bash
# ============================================================
# 获取当前机器的 Device Code
# 用法: bash get_device_code.sh
# 输出: 硬件信息和 Device Code，复制到 camerad.cc 白名单即可
# ============================================================

echo "=========================================="
echo "  Device Code 获取工具"
echo "=========================================="
echo ""

# --- Machine ID ---
if [ -f /etc/machine-id ]; then
    MACHINE_ID=$(tr -d '[:space:]' < /etc/machine-id)
    echo "Machine ID:  $MACHINE_ID"
else
    MACHINE_ID=""
    echo "Machine ID:  (未获取)"
fi

# --- BIOS UUID ---
if [ -r /sys/class/dmi/id/product_uuid ]; then
    BIOS_UUID=$(tr -d '[:space:]' < /sys/class/dmi/id/product_uuid)
    echo "BIOS UUID:   $BIOS_UUID"
else
    BIOS_UUID=""
    echo "BIOS UUID:   (未获取，需要root权限)"
fi

# --- Hostname ---
HOSTNAME=$(hostname)
echo "Hostname:    $HOSTNAME"

# --- MAC 地址 ---
MAC_ADDR=""

# 方法1: 从 /sys/class/net 读取（优先，过滤虚拟网卡）
# 虚拟网卡（docker0/veth*/virbr*/br-*等）无 device 目录，MAC不稳定
for iface in /sys/class/net/*; do
    name=$(basename "$iface")
    if [ "$name" = "lo" ] || [ "$name" = "." ]; then continue; fi
    # 跳过虚拟网卡：物理网卡有 device 软链接，虚拟网卡没有
    if [ ! -e "$iface/device" ]; then continue; fi
    addr=$(tr -d '[:space:]' < "$iface/address" 2>/dev/null)
    if [ -n "$addr" ] && [ "$addr" != "00:00:00:00:00:00" ]; then
        MAC_ADDR="$addr"
        break
    fi
done

# 方法2: 从 /proc/net/arp 备选
if [ -z "$MAC_ADDR" ]; then
    while IFS= read -r line; do
        echo "$line" | grep -q "IP address" && continue
        mac=$(echo "$line" | awk '{print $4}')
        dev=$(echo "$line" | awk '{print $6}')
        if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ] && [ "$dev" != "lo" ]; then
            MAC_ADDR="$mac"
            break
        fi
    done < /proc/net/arp 2>/dev/null
fi

echo "MAC地址:     ${MAC_ADDR:-"(未获取)"}"
echo ""

# --- 计算 Device Code ---
# 统一使用 machine_id + hostname + mac（与 C++ 代码一致，不依赖 BIOS UUID）
COMBINED="${MACHINE_ID}|${HOSTNAME}|${MAC_ADDR}"

DEVICE_CODE=$(python3 -c "
import sys

combined = '''$COMBINED'''

h = 14695981039346656037
for c in combined:
    h ^= ord(c)
    h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF

print(format(h, '016X'))
")

echo "=========================================="
echo "Device Code:  $DEVICE_CODE"
echo "=========================================="
echo ""
echo "复制上面这行 Device Code 到 tools/webcam/camerad.cc 的 AUTHORIZED_DEVICE_CODES 列表中即可。"
