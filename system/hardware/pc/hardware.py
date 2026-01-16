import random
import shutil
import os
import subprocess
import glob
from cereal import log
from openpilot.system.hardware.base import HardwareBase, ThermalConfig, ThermalZone
# from openpilot.system.hardware.lpa.base import LPABase  # 注释掉这一行

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength

class Pc(HardwareBase):
  def get_os_version(self):
    return None

  def get_device_type(self):
    return "pc"

  def reboot(self, reason=None):
    subprocess.check_output(["sudo", "reboot"])

  def uninstall(self):
    print("uninstall")

  def get_imei(self, slot):
    return f"{random.randint(0, 1 << 32):015d}"

  def get_serial(self):
    return "cccccccc"

  def get_network_info(self):
    return None

  def get_network_type(self):
    return NetworkType.wifi

  def get_memory_usage_percent(self):
    try:
        with open('/proc/meminfo') as f:
            meminfo = {}
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    key = parts[0].rstrip(':')
                    value = int(parts[1])  # kB
                    meminfo[key] = value

        # 基于您的系统数据: MemTotal: 13196996 kB, MemAvailable: 11451184 kB
        total = meminfo.get('MemTotal', 0)
        available = meminfo.get('MemAvailable', meminfo.get('MemFree', 0))
        if total > 0:
            used_percent = ((total - available) / total) * 100
            return int(used_percent)
    except:
        pass
    return 0

  def get_free_space_percent(self):
    try:
      # 获取根分区使用情况
      usage = shutil.disk_usage('/')
      free_percent = (usage.free / usage.total) * 100
      return free_percent
    except:
      return 0.0

  def get_sim_info(self):
    return {
      'sim_id': '',
      'mcc_mnc': None,
      'network_type': ["Unknown"],
      'sim_state': ["ABSENT"],
      'data_connected': False
    }

  def get_sim_lpa(self):
    # PC环境不支持SIM LPA，直接返回None或抛异常
    return None

  def get_network_strength(self, network_type):
    return NetworkStrength.unknown

  def get_current_power_draw(self):
    return 0

  def get_som_power_draw(self):
    return 0

  def shutdown(self):
    os.system("sudo poweroff")

  def get_thermal_config(self):
    cpu_zones = []
    gpu_zones = []

    # 扫描所有 hwmon 目录
    for hwmon in glob.glob('/sys/class/hwmon/hwmon*'):
      name_path = os.path.join(hwmon, 'name')
      if not os.path.exists(name_path):
        continue

      with open(name_path) as f:
        dev_name = f.read().strip().lower()

      # 判断这是不是 CPU 或 GPU 的温度源
      is_cpu = 'k10temp' in dev_name or 'coretemp' in dev_name
      is_gpu = 'amdgpu' in dev_name

      # 查找该 hwmon 下的所有温度传感器
      for temp_input in glob.glob(os.path.join(hwmon, 'temp*_input')):

        # 获取 temp label
        label_path = temp_input.replace('_input', '_label')
        if os.path.exists(label_path):
          with open(label_path) as f:
            zone_type = f.read().strip()
        else:
          zone_type = os.path.basename(temp_input)

        # 创建 ThermalZone 实例（保持兼容）
        zone = ThermalZone(zone_type)

        # 重写 read()，直接读取 hwmon 文件，而不用 thermal_zone*
        zone.read = (lambda p=temp_input: int(open(p).read().strip()) / 1000.0)

        # 分类
        if is_cpu:
          cpu_zones.append(zone)

        if is_gpu:
          gpu_zones.append(zone)

    # 如果啥都没扫到，为兼容旧逻辑保底
    if not cpu_zones:
      cpu_zones.append(ThermalZone('thermal_zone0'))

    # CPU 显示 GPU 温度
    final_cpu = cpu_zones if cpu_zones else gpu_zones
    final_gpu = gpu_zones if gpu_zones else None

    return ThermalConfig(
      cpu=final_cpu,
      gpu=final_gpu,
      memory=None,
      pmic=None
    )

  def set_screen_brightness(self, percentage):
    pass

  def get_screen_brightness(self):
    return 0

  def set_power_save(self, powersave_enabled):
    pass

  def get_gpu_usage_percent(self):
    import glob
    max_usage = 0
    try:
        gpu_busy_files = glob.glob('/sys/class/drm/card*/device/gpu_busy_percent')
        for gpu_file in gpu_busy_files:
            try:
                with open(gpu_file) as f:
                    usage = int(f.read().strip())
                    max_usage = max(max_usage, usage)
            except:
                continue
    except Exception:
        pass
    return max_usage

  def get_modem_temperatures(self):
    return []

  def get_nvme_temperatures(self):
    return []

  def initialize_hardware(self):
    pass

  def get_networks(self):
    return None
