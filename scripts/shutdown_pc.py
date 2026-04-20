#!/usr/bin/env python3
"""
关机脚本，用于通过UI设置触发系统关机
此脚本会检测运行环境（PC或真实硬件），并在不同环境下执行适当的关机操作
"""
import os
import sys
import subprocess
import platform
from typing import Union


def is_pc():
    """判断是否在PC环境运行"""
    # 检测是否设置了环境变量表示PC模式
    return os.getenv("COMMA_PLUG_TYPE") == "pc" or os.getenv("USER") == "liruifeng"


def shutdown_system():
    """执行系统关机操作"""
    print("正在关闭系统...")
    
    try:
        if is_pc():
            # 在PC上使用systemctl而不是poweroff命令
            print("检测到PC环境，使用systemctl进行关机")
            subprocess.run(['sudo', 'systemctl', 'poweroff'], check=True)
        else:
            # 在真实硬件上使用poweroff命令
            print("检测到硬件环境，使用poweroff命令")
            subprocess.run(['sudo', 'poweroff'], check=True)
    except subprocess.CalledProcessError as e:
        print(f"关机命令执行失败: {e}")
        # 尝试使用另一种方法
        try:
            if is_pc():
                subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
            else:
                subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
        except subprocess.CalledProcessError as e2:
            print(f"备用关机命令也失败了: {e2}")
            sys.exit(1)
    except KeyboardInterrupt:
        print("关机操作被用户取消")
        sys.exit(0)


def main():
    """主函数"""
    if os.geteuid() != 0:
        print("此脚本需要root权限，请使用sudo运行或在UI中点击关机按钮")
        
    shutdown_system()


if __name__ == "__main__":
    main()