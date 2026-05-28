#!/usr/bin/env python3
import os
import time
import ctypes
import math
import select
import threading
import glob
import smbus2
import platform

import cereal.messaging as messaging
from cereal import log
from cereal.services import SERVICE_LIST
from openpilot.common.util import sudo_write
from openpilot.common.realtime import config_realtime_process, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.gpio import gpiochip_get_ro_value_fd, gpioevent_data
from openpilot.system.hardware import HARDWARE

from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.lsm6ds3_accel import LSM6DS3_Accel
from openpilot.system.sensord.sensors.lsm6ds3_gyro import LSM6DS3_Gyro
from openpilot.system.sensord.sensors.lsm6ds3_temp import LSM6DS3_Temp
from openpilot.system.sensord.sensors.mmc5603nj_magn import MMC5603NJ_Magn

DEFAULT_I2C_BUS_IMU = 1
LSM6DS3_I2C_ADDRS = (0x6A, 0x6B)
LSM6DS3_WHO_AM_I_REG = 0x0F
LSM6DS3_WHO_AM_I_IDS = (0x69, 0x6A)
MMC5603_I2C_ADDR = 0x30
MMC5603_WHO_AM_I_REG = 0x39
MMC5603_WHO_AM_I_IDS = (0x10,)

# CH347 库文件路径配置
def get_ch347_lib_path():
    """获取 CH347 库文件路径，优先从项目 third_party 目录查找"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(script_dir))
    lib_dir = os.path.join(project_root, "third_party", "ch347", "lib")

    # 根据系统架构选择对应的库文件
    machine = platform.machine().lower()
    if machine in ("x86_64", "amd64"):
        arch_dir = "x64"
    elif machine.startswith("arm"):
        if machine.endswith("v7l") or machine.endswith("hf"):
            arch_dir = "arm-gnueabihf"
        else:
            arch_dir = "arm-gnueabi"
    elif machine.startswith("aarch64"):
        arch_dir = "aarch64"
    else:
        arch_dir = "x86"

    lib_path = os.path.join(lib_dir, arch_dir, "dynamic", "libch347.so")

    # 如果项目目录没有，回退到默认路径
    if not os.path.exists(lib_path):
        lib_path = os.path.join(lib_dir, "x64", "dynamic", "libch347.so")

    return lib_path

CH347_DEFAULT_LIB_PATH = get_ch347_lib_path()
CH347_WHO_AM_I_REG = 0x0F
CH347_LSM6_ADDRS = (0x6B, 0x6A)  # 优先 0x6B
CH347_WHO_AM_I_IDS = (0x69, 0x6A)


class CH347LSM6:
    def __init__(self, dev_path: str, lib_path: str) -> None:
        self.dev_path = dev_path
        self.lib = ctypes.CDLL(lib_path)
        self.fd = -1
        self.addr = None
        self.source = None

    def _stream_read(self, write_bytes: list[int], read_len: int) -> bytes:
        wbuf = (ctypes.c_ubyte * len(write_bytes))(*write_bytes)
        rbuf = (ctypes.c_ubyte * read_len)()
        ack = ctypes.c_ubyte()
        ok = self.lib.CH347StreamI2C_RetAck(self.fd, len(write_bytes), wbuf, read_len, rbuf, ctypes.byref(ack))
        if not ok:
            raise OSError(f"CH347 I2C read failed, ack={ack.value}")
        return bytes(rbuf)

    def _stream_write(self, write_bytes: list[int]) -> None:
        wbuf = (ctypes.c_ubyte * len(write_bytes))(*write_bytes)
        ok = self.lib.CH347StreamI2C(self.fd, len(write_bytes), wbuf, 0, None)
        if not ok:
            raise OSError("CH347 I2C write failed")

    def read_u8(self, reg: int) -> int:
        return self._stream_read([self.addr << 1, reg], 1)[0]

    def write_u8(self, reg: int, val: int) -> None:
        self._stream_write([self.addr << 1, reg, val])

    def read_block(self, start_reg: int, length: int) -> bytes:
        # 使用多字节连续读取，比单字节循环读取高效得多
        # 组合：写地址 + 读取数据
        write_bytes = [self.addr << 1 | 0x00, start_reg]
        wbuf = (ctypes.c_ubyte * len(write_bytes))(*write_bytes)
        rbuf = (ctypes.c_ubyte * length)()
        ack = ctypes.c_ubyte()

        # 先写寄存器地址
        ok = self.lib.CH347StreamI2C_RetAck(self.fd, len(write_bytes), wbuf, 0, None, ctypes.byref(ack))
        if not ok:
            raise OSError(f"CH347 I2C block write address failed, ack={ack.value}")

        # 然后连续读取数据
        wbuf_read = (ctypes.c_ubyte * 1)(self.addr << 1 | 0x01)
        ok = self.lib.CH347StreamI2C_RetAck(self.fd, 1, wbuf_read, length, rbuf, ctypes.byref(ack))
        if not ok:
            raise OSError(f"CH347 I2C block read data failed, ack={ack.value}")

        return bytes(rbuf)

    def open(self) -> None:
        self.fd = self.lib.CH347OpenDevice(self.dev_path.encode())
        if self.fd < 0:
            raise OSError(f"failed to open {self.dev_path}")

        if not self.lib.CH34xSetTimeout(self.fd, 2000, 2000):
            raise OSError("CH34xSetTimeout failed")

        if not self.lib.CH347I2C_Set(self.fd, 0x01):  # 100kHz (更稳定)
            raise OSError("CH347I2C_Set failed")

        self.lib.CH347I2C_SetIgnoreNack(self.fd, 1)
        self.lib.CH347I2C_SetStretch(self.fd, True)

        time.sleep(0.02)

        detected_sensors = []
        for _ in range(8):
            for addr in CH347_LSM6_ADDRS:
                try:
                    who = self._stream_read([addr << 1, CH347_WHO_AM_I_REG], 1)[0]
                    if who in CH347_WHO_AM_I_IDS:
                        detected_sensors.append((addr, who))
                        self.addr = addr
                        self.source = log.SensorEventData.SensorSource.lsm6ds3trc if who == 0x6A else log.SensorEventData.SensorSource.lsm6ds3
                        cloudlog.info(f"CH347 LSM6DS3 detected at addr=0x{addr:02X}, who=0x{who:02X}")
                        return
                except Exception:
                    continue
            time.sleep(0.02)

        # 详细日志：未检测到传感器
        if detected_sensors:
            cloudlog.error(f"CH347 LSM6DS3 detected but WHO_AM_I mismatch: {detected_sensors}")
        else:
            cloudlog.error(f"CH347 LSM6DS3 not detected on any of these addresses: {CH347_LSM6_ADDRS}")
        raise OSError("CH347 LSM6DS3 not detected on 0x6A/0x6B")

    def init_sensor(self) -> None:
        # Reset and configure accel+gyro at 104Hz, +-2g / +-245dps.
        self.write_u8(0x12, 0x01)
        time.sleep(0.1)
        self.write_u8(0x12, 0x04)
        self.write_u8(0x10, 0x40)
        self.write_u8(0x11, 0x40)

    def shutdown(self) -> None:
        if self.fd >= 0:
            try:
                self.write_u8(0x10, 0x00)
                self.write_u8(0x11, 0x00)
            except Exception:
                pass
            self.lib.CH347CloseDevice(self.fd)
            self.fd = -1

def detect_ch347_backend() -> tuple[str, str] | None:
    dev = os.getenv("SENSORD_CH347_DEV")
    if not dev:
        devs = sorted(glob.glob("/dev/ch34x_pis*"))
        if not devs:
            cloudlog.warning("No CH347 devices found in /dev/ch34x_pis*")
            return None
        # 优先使用第一个设备，但记录所有可用的
        cloudlog.info(f"Found CH347 devices: {devs}, using {devs[0]}")
        dev = devs[0]

    lib_path = os.getenv("SENSORD_CH347_LIB", CH347_DEFAULT_LIB_PATH)

    # 验证设备存在
    if not os.path.exists(dev):
        cloudlog.error(f"CH347 device does not exist: {dev}")
        return None

    # 验证库文件存在
    if not os.path.exists(lib_path):
        cloudlog.error(f"CH347 library does not exist: {lib_path}")
        return None

    cloudlog.info(f"CH347 backend: dev={dev}, lib={lib_path}")
    return dev, lib_path

def ch347_loop(dev_path: str, lib_path: str, event: threading.Event) -> None:
    sensor = CH347LSM6(dev_path, lib_path)
    pm = messaging.PubMaster(["accelerometer", "gyroscope", "temperatureSensor"])
    rk = Ratekeeper(104, print_delay_threshold=None)  # 104Hz 匹配传感器配置
    temp_ticks = 0
    last_log_t = 0.0
    last_acc_v = None
    last_gyro_v = None
    try:
        sensor.open()
        sensor.init_sensor()
        cloudlog.info(f"Using CH347 backend: dev={dev_path}")

        last_gyro_log_t = 0.0

        while not event.is_set():
            status = sensor.read_u8(0x1E)

            # 读取加速度计（如果数据就绪）
            if status & 0x01:
                b = sensor.read_block(0x28, 6)
                x = Sensor.parse_16bit(b[0], b[1])
                y = Sensor.parse_16bit(b[2], b[3])
                z = Sensor.parse_16bit(b[4], b[5])
                scale = 9.81 * 2.0 / (1 << 15)
                last_acc_v = [y * scale, -x * scale, z * scale]

                msg = messaging.new_message("accelerometer", valid=True)
                msg.accelerometer.version = 1
                msg.accelerometer.sensor = 1
                msg.accelerometer.type = 1
                msg.accelerometer.source = sensor.source
                msg.accelerometer.timestamp = msg.logMonoTime  # 使用logMonoTime而不是time.time_ns()
                acc = msg.accelerometer.init("acceleration")
                acc.v = last_acc_v
                acc.status = 1
                pm.send("accelerometer", msg)

            # 读取陀螺仪（如果数据就绪）- 即使加速度计也读取了，也要检查陀螺仪
            # 因为两者可能都准备好数据
            if status & 0x02:
                b = sensor.read_block(0x22, 6)
                x = Sensor.parse_16bit(b[0], b[1])
                y = Sensor.parse_16bit(b[2], b[3])
                z = Sensor.parse_16bit(b[4], b[5])
                scale = (8.75 / 1000.0) * (math.pi / 180.0)
                last_gyro_v = [y * scale, -x * scale, z * scale]
                msg = messaging.new_message("gyroscope", valid=True)
                msg.gyroscope.version = 2
                msg.gyroscope.sensor = 5
                msg.gyroscope.type = 16
                msg.gyroscope.source = sensor.source
                msg.gyroscope.timestamp = msg.logMonoTime  # 使用logMonoTime而不是time.time_ns()
                gyro = msg.gyroscope.init("gyroUncalibrated")
                gyro.v = last_gyro_v
                gyro.status = 1
                pm.send("gyroscope", msg)

            temp_ticks += 1

            # 每秒打印一次数据
            current_t = time.time()
            if current_t - last_log_t >= 1.0:
                # if last_acc_v:
                #     print(f"[sensord] ACC: [{last_acc_v[0]:7.2f}, {last_acc_v[1]:7.2f}, {last_acc_v[2]:7.2f}] m/s²", flush=True)
                # if last_gyro_v:
                #     print(f"[sensord] GYRO: [{last_gyro_v[0]:7.3f}, {last_gyro_v[1]:7.3f}, {last_gyro_v[2]:7.3f}] rad/s", flush=True)
                last_log_t = current_t
            if temp_ticks >= 10:
                temp_ticks = 0
                tb = sensor.read_block(0x20, 2)
                raw_t = Sensor.parse_16bit(tb[0], tb[1])
                tscale = 16.0 if sensor.source == log.SensorEventData.SensorSource.lsm6ds3 else 256.0

                msg = messaging.new_message("temperatureSensor", valid=True)
                msg.temperatureSensor.version = 1
                msg.temperatureSensor.source = sensor.source
                msg.temperatureSensor.timestamp = msg.logMonoTime
                msg.temperatureSensor.temperature = 25 + (raw_t / tscale)
                pm.send("temperatureSensor", msg)

            rk.keep_time()
    finally:
        sensor.shutdown()


def detect_imu_i2c_bus() -> int:
  bus_override = os.getenv("SENSORD_I2C_BUS")
  if bus_override is not None:
    try:
      bus = int(bus_override)
      cloudlog.info(f"Using IMU bus from SENSORD_I2C_BUS={bus}")
      return bus
    except ValueError:
      cloudlog.error(f"Invalid SENSORD_I2C_BUS={bus_override}, falling back to auto-detect")

  candidates = []
  for node in sorted(glob.glob("/dev/i2c-*")):
    try:
      candidates.append(int(node.rsplit("-", 1)[1]))
    except ValueError:
      continue

  if DEFAULT_I2C_BUS_IMU not in candidates:
    candidates.insert(0, DEFAULT_I2C_BUS_IMU)
  else:
    candidates = [DEFAULT_I2C_BUS_IMU] + [b for b in candidates if b != DEFAULT_I2C_BUS_IMU]

  for bus in candidates:
    try:
      with smbus2.SMBus(bus) as i2c:
        for addr in LSM6DS3_I2C_ADDRS:
          try:
            chip_id = i2c.read_byte_data(addr, LSM6DS3_WHO_AM_I_REG)
          except OSError as e:
            cloudlog.warning(f"Failed to read LSM6DS3 on bus {bus} at addr 0x{addr:02X}: {e}")
            continue

          if chip_id in LSM6DS3_WHO_AM_I_IDS:
            cloudlog.info(f"Detected LSM6DS3 on /dev/i2c-{bus}, addr=0x{addr:02X}, chip=0x{chip_id:02X}")
            return bus
    except OSError as e:
      cloudlog.warning(f"Failed to open I2C bus {bus}: {e}")
      continue

  cloudlog.warning(f"Unable to auto-detect LSM6DS3, falling back to /dev/i2c-{DEFAULT_I2C_BUS_IMU}")
  return DEFAULT_I2C_BUS_IMU


def has_imu_interrupt() -> bool:
  try:
    fd = gpiochip_get_ro_value_fd("sensord", 0, 84)
    os.close(fd)
    return True
  except Exception:
    cloudlog.warning("IMU GPIO interrupt unavailable, using polling fallback for accel/gyro")
    return False


def is_magnetometer_present(bus: int) -> bool:
  # Allow forcing behavior for bring-up and testing.
  env_val = os.getenv("SENSORD_ENABLE_MAG")
  if env_val is not None:
    enabled = env_val.lower() in {"1", "true", "yes", "on"}
    cloudlog.info(f"Magnetometer forced {'enabled' if enabled else 'disabled'} by SENSORD_ENABLE_MAG={env_val}")
    return enabled

  try:
    with smbus2.SMBus(bus) as i2c:
      chip_id = i2c.read_byte_data(MMC5603_I2C_ADDR, MMC5603_WHO_AM_I_REG)
      if chip_id in MMC5603_WHO_AM_I_IDS:
        cloudlog.info(f"Detected MMC5603 on /dev/i2c-{bus}, chip=0x{chip_id:02X}")
        return True
  except OSError:
    pass

  cloudlog.warning("MMC5603 not detected, magnetometer publisher disabled")
  return False

def interrupt_loop(sensors: list[tuple[Sensor, str, bool]], event) -> None:
  pm = messaging.PubMaster([service for sensor, service, interrupt in sensors if interrupt])

  # Requesting both edges as the data ready pulse from the lsm6ds sensor is
  # very short (75us) and is mostly detected as falling edge instead of rising.
  # So if it is detected as rising the following falling edge is skipped.
  fd = gpiochip_get_ro_value_fd("sensord", 0, 84)

  # Configure IRQ affinity
  irq_path = "/proc/irq/336/smp_affinity_list"
  if not os.path.exists(irq_path):
    irq_path = "/proc/irq/335/smp_affinity_list"
  if os.path.exists(irq_path):
    sudo_write('1\n', irq_path)

  offset = time.time_ns() - time.monotonic_ns()

  poller = select.poll()
  poller.register(fd, select.POLLIN | select.POLLPRI)
  while not event.is_set():
    events = poller.poll(100)
    if not events:
      cloudlog.error("poll timed out")
      continue
    if not (events[0][1] & (select.POLLIN | select.POLLPRI)):
      cloudlog.error("no poll events set")
      continue

    dat = os.read(fd, ctypes.sizeof(gpioevent_data)*16)
    evd = gpioevent_data.from_buffer_copy(dat)

    cur_offset = time.time_ns() - time.monotonic_ns()
    if abs(cur_offset - offset) > 10 * 1e6:  # ms
      cloudlog.warning(f"time jumped: {cur_offset} {offset}")
      offset = cur_offset
      continue

    ts = evd.timestamp - cur_offset
    for sensor, service, interrupt in sensors:
      if interrupt:
        try:
          evt = sensor.get_event(ts)
          if not sensor.is_data_valid():
            continue
          msg = messaging.new_message(service, valid=True)
          setattr(msg, service, evt)
          pm.send(service, msg)
        except Sensor.DataNotReady:
          pass
        except Exception:
          cloudlog.exception(f"Error processing {service}")


def polling_loop(sensor: Sensor, service: str, event: threading.Event, needs_timestamp: bool = False) -> None:
  pm = messaging.PubMaster([service])
  rk = Ratekeeper(SERVICE_LIST[service].frequency, print_delay_threshold=None)
  while not event.is_set():
    try:
      evt = sensor.get_event(time.time_ns()) if needs_timestamp else sensor.get_event()
      if not sensor.is_data_valid():
        continue
      msg = messaging.new_message(service, valid=True)
      setattr(msg, service, evt)
      pm.send(service, msg)
    except Exception:
      cloudlog.exception(f"Error in {service} polling loop")
    rk.keep_time()

def main() -> None:
  config_realtime_process([1, ], 1)

  ch347_backend = detect_ch347_backend()
  if ch347_backend is not None:
    dev_path, lib_path = ch347_backend
    exit_event = threading.Event()
    try:
      ch347_loop(dev_path, lib_path, exit_event)
      return
    except KeyboardInterrupt:
      return
    except OSError as e:
      # 区分设备错误和其他错误
      if "not detected" in str(e):
        cloudlog.error(f"CH347 sensor detection failed: {e}")
      elif "failed to open" in str(e):
        cloudlog.error(f"CH347 device not found or not accessible: {e}")
      else:
        cloudlog.exception(f"CH347 I2C error: {e}")
      cloudlog.info("Falling back to smbus path")
    except Exception:
      cloudlog.exception("CH347 backend failed with unexpected error, falling back to smbus path")

  i2c_bus_imu = detect_imu_i2c_bus()
  interrupt_available = has_imu_interrupt()
  magnetometer_available = is_magnetometer_present(i2c_bus_imu)

  sensors_cfg = [
    (LSM6DS3_Accel(i2c_bus_imu), "accelerometer", True),
    (LSM6DS3_Gyro(i2c_bus_imu), "gyroscope", True),
    (LSM6DS3_Temp(i2c_bus_imu), "temperatureSensor", False),
  ]
  if HARDWARE.get_device_type() == "tizi":
    sensors_cfg.append(
      (MMC5603NJ_Magn(i2c_bus_imu), "magnetometer", False),
    )

  # Reset sensors
  for sensor, _, _ in sensors_cfg:
    try:
      sensor.reset()
    except Exception:
      cloudlog.exception(f"Error initializing {sensor} sensor")

  # Initialize sensors
  initialized_sensors = []
  exit_event = threading.Event()
  threads = []
  if any(interrupt for _, _, interrupt in sensors_cfg):
    threads.append(threading.Thread(target=interrupt_loop, args=(sensors_cfg, exit_event), daemon=True))

  for sensor, service, interrupt in sensors_cfg:
    try:
      sensor.init()
      initialized_sensors.append(sensor)
      if not interrupt:
        # Start polling thread for sensors without interrupts
        needs_timestamp = service in {"accelerometer", "gyroscope"}
        threads.append(threading.Thread(
          target=polling_loop,
          args=(sensor, service, exit_event, needs_timestamp),
          daemon=True
        ))
    except Exception:
      cloudlog.exception(f"Error initializing {service} sensor")

  try:
    for t in threads:
      t.start()
    while any(t.is_alive() for t in threads):
      time.sleep(1)
  except KeyboardInterrupt:
    pass
  finally:
    exit_event.set()
    for t in threads:
      if t.is_alive():
        t.join()

    for sensor in initialized_sensors:
      try:
        sensor.shutdown()
      except Exception:
        cloudlog.exception("Error shutting down sensor")

if __name__ == "__main__":
  main()
