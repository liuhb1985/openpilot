import subprocess
import os
import time
import logging

def is_panda_connected():
    try:
        # 通过lsusb命令检查panda设备（不区分大小写）
        result = subprocess.check_output(
            "lsusb | grep -i 'Panda'",  # -i 表示不区分大小写
            shell=True,
            stderr=subprocess.STDOUT,
            text=True
        )
        return len(result.strip()) > 0
    except subprocess.CalledProcessError:
        return False
    except Exception as e:
        print(f"检查设备时出错: {e}")
        return False  # 出现其他错误也视为设备未连接

def main():
    logging.basicConfig(filename='/data/openpilot/panda_monitor.log',
	                    level=logging.INFO,
						format='%(asctime)s [%(levelname)s] %(message)s')
    logging.info("Panda Monitor 启动")

    offline_threshold = 60  # 离线阈值（秒），3分钟=300秒
    offline_seconds = 0      # 当前累计离线时间
    check_interval = 10      # 检查间隔（秒）

    while True:
        if is_panda_connected():
            # 设备在线，重置离线计时
            if offline_seconds > 0:
                logging.info("Panda已重新连接，重置计时")
                print("Panda已重新连接，重置计时")
                offline_seconds = 0
        else:
            # 设备离线，累计时间
            offline_seconds += check_interval
            print(f"Panda离线中，已持续 {offline_seconds} 秒")

            # 达到阈值执行关机
            if offline_seconds >= offline_threshold:
                logging.info("Panda离线已达5分钟，执行关机")
                print("Panda离线已达5分钟，执行关机")
                try:
                    # 尝试使用不需要密码的关机方式
                    os.system("shutdown -h now")
                except Exception as e:
                    print(f"关机失败: {e}")
                break

        time.sleep(check_interval)  # 每分钟检查一次

if __name__ == "__main__":
    main()
