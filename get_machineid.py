#!/usr/bin/env python3
import uuid
import glob
import subprocess
import os

def get_machine_id():
    """获取 machine-id (推荐，最稳定)"""
    path = '/etc/machine-id'
    if os.path.exists(path):
        with open(path) as f:
            return f.read().strip()
    return None

def get_bios_uuid():
    """获取 BIOS UUID (需要 dmidecode)"""
    try:
        result = subprocess.run(
            ['sudo', 'dmidecode', '-s', 'system-uuid'],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout.strip()
    except Exception:
        return None

def get_board_serial():
    """获取主板序列号 (需要 dmidecode)"""
    try:
        result = subprocess.run(
            ['sudo', 'dmidecode', '-s', 'baseboard-serial-number'],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout.strip()
    except Exception:
        return None

def get_cpu_id():
    """获取 CPU ID (Linux下不可靠)"""
    try:
        result = subprocess.run(
            ['cat', '/proc/cpuinfo'],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.split('\n'):
            if 'Serial' in line:
                return line.split(':')[1].strip()
    except Exception:
        pass
    return None

def get_mac_address():
    """获取第一个网卡的MAC地址"""
    mac = ':'.join(f'{(uuid.getnode() >> i) & 0xff:02x}' for i in range(0, 48, 8))
    return mac

def get_device_code():
    """生成设备码 (基于多个硬件标识组合)"""
    parts = []
    mid = get_machine_id()
    if mid:
        parts.append(mid)
    uuid_bios = get_bios_uuid()
    if uuid_bios:
        parts.append(uuid_bios)
    board_serial = get_board_serial()
    if board_serial:
        parts.append(board_serial)
    mac = get_mac_address()
    parts.append(mac.replace(':', ''))

    # 用第一部分的哈希作为设备码
    combined = '_'.join(parts)
    device_code = uuid.uuid5(uuid.NAMESPACE_DNS, combined).hex[:16].upper()
    return device_code, combined

if __name__ == "__main__":
    print("=== 硬件唯一码 ===")
    print(f"Machine ID:     {get_machine_id()}")
    print(f"BIOS UUID:      {get_bios_uuid()}")
    print(f"主板序列号:     {get_board_serial()}")
    print(f"MAC地址:        {get_mac_address()}")

    device_code, _ = get_device_code()
    print(f"\n=== 设备码 ===")
    print(f"Device Code:    {device_code}")
    print(f"\n可用于加密绑定")
