import os
import random
import subprocess

from cereal import log
from openpilot.system.hardware.base import HardwareBase, ThermalConfig

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength

# PC Webcam Configuration
WEBCAM_W = int(os.getenv("WEBCAM_W", "1920"))
WEBCAM_H = int(os.getenv("WEBCAM_H", "1080"))
WEBCAM_FOCAL = int(os.getenv("WEBCAM_FOCAL", "2520")) # Default for many standard webcams

class Pc(HardwareBase):
  def get_os_version(self):
    return subprocess.check_output(["uname", "-rv"], encoding='utf8').strip()

  def get_device_type(self):
    return "pc"

  def get_sound_card_online(self):
    return True

  def reboot(self, reason=None):
    print(f"REBOOT! reason: {reason}")
    # os.system("sudo reboot")

  def uninstall(self):
    print("uninstall")

  def get_imei(self, slot):
    return "%015d" % random.randint(0, 1 << 32)

  def get_serial(self):
    return "cccccccc"

  def get_subscriber_info(self):
    return ""

  def get_network_info(self):
    return None

  def get_network_type(self):
    return NetworkType.wifi

  def get_sim_info(self):
    return {
      'sim_id': '',
      'mcc_mnc': None,
      'network_type': ["Unknown"],
      'sim_state': ["ABSENT"],
      'data_connected': False
    }

  def get_network_strength(self, network_type):
    return NetworkStrength.unknown

  def get_current_power_draw(self):
    return 0

  def get_som_power_draw(self):
    return 0

  def shutdown(self):
    print("SHUTDOWN!")
    # os.system("sudo shutdown -h now")

  def get_thermal_config(self):
    # Try to find CPU temperature from hwmon
    cpu_temp = 0
    try:
      for i in range(10):
        path = f"/sys/class/hwmon/hwmon{i}/temp1_input"
        if os.path.exists(path):
          with open(path, "r") as f:
            cpu_temp = int(f.read().strip()) / 1000.0
            break
    except Exception:
      pass

    return ThermalConfig(cpu=((cpu_temp,), 1), gpu=((0,), 1), mem=(0, 1), bat=(0, 1), ambient=(0, 1), pmic=((0,), 1))

  def set_screen_brightness(self, percentage):
    pass

  def get_screen_brightness(self):
    return 100

  def set_power_save(self, powersave_enabled):
    pass

  def get_gpu_usage_percent(self):
    # Try nvidia-smi for NVIDIA GPUs
    try:
      val = subprocess.check_output(["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"], encoding='utf8')
      return int(val.strip())
    except Exception:
      pass
    return 0

  def get_modem_temperatures(self):
    return []

  def get_nvme_temperatures(self):
    # Try to read NVMe temp if smartctl is available and we have permission
    return []

  def initialize_hardware(self):
    pass

  def get_networks(self):
    return None
