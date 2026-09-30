#!/usr/bin/env python3
import os
import time
import ctypes
import math
import select
import threading
import glob

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.utils import sudo_write
from openpilot.common.i2c import SMBus
from openpilot.common.realtime import config_realtime_process, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.gpio import gpiochip_get_ro_value_fd, gpioevent_data

from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.lsm6ds3_accel import LSM6DS3_Accel
from openpilot.system.sensord.sensors.lsm6ds3_gyro import LSM6DS3_Gyro
from openpilot.system.sensord.sensors.lsm6ds3_temp import LSM6DS3_Temp

DEFAULT_I2C_BUS_IMU = 1
LSM6DS3_I2C_ADDRS = (0x6A, 0x6B)
LSM6DS3_WHO_AM_I_REG = 0x0F
LSM6DS3_WHO_AM_I_IDS = (0x69, 0x6A)


# *** CH347 (USB-I2C bridge) backend — mirrors /home/peter/openpilot ***

def get_ch347_lib_path():
  """CH347 library path, preferring the project third_party directory."""
  project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

  machine = os.uname().machine.lower()
  if machine in ("x86_64", "amd64"):
    arch_dir = "x64"
  elif machine.startswith("arm"):
    arch_dir = "arm-gnueabihf" if (machine.endswith("v7l") or machine.endswith("hf")) else "arm-gnueabi"
  elif machine.startswith("aarch64"):
    arch_dir = "aarch64"
  else:
    arch_dir = "x86"

  lib_path = os.path.join(project_root, "third_party", "ch347", "lib", arch_dir, "dynamic", "libch347.so")
  if not os.path.exists(lib_path):
    lib_path = os.path.join(project_root, "third_party", "ch347", "lib", "x64", "dynamic", "libch347.so")
  return lib_path

CH347_DEFAULT_LIB_PATH = get_ch347_lib_path()
CH347_WHO_AM_I_REG = 0x0F
CH347_LSM6_ADDRS = (0x6B, 0x6A)  # prefer 0x6B
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
    for attempt in range(3):
      try:
        return self._stream_read([self.addr << 1, reg], 1)[0]
      except OSError:
        if attempt == 2:
          raise
        time.sleep(0.001)

  def write_u8(self, reg: int, val: int) -> None:
    for attempt in range(3):
      try:
        self._stream_write([self.addr << 1, reg, val])
        return
      except OSError:
        if attempt == 2:
          raise
        time.sleep(0.001)

  def read_block(self, start_reg: int, length: int) -> bytes:
    for attempt in range(3):
      try:
        # combined: write register address, then contiguous read
        write_bytes = [self.addr << 1 | 0x00, start_reg]
        wbuf = (ctypes.c_ubyte * len(write_bytes))(*write_bytes)
        rbuf = (ctypes.c_ubyte * length)()
        ack = ctypes.c_ubyte()

        ok = self.lib.CH347StreamI2C_RetAck(self.fd, len(write_bytes), wbuf, 0, None, ctypes.byref(ack))
        if not ok:
          raise OSError(f"CH347 I2C block write address failed, ack={ack.value}")

        wbuf_read = (ctypes.c_ubyte * 1)(self.addr << 1 | 0x01)
        ok = self.lib.CH347StreamI2C_RetAck(self.fd, 1, wbuf_read, length, rbuf, ctypes.byref(ack))
        if not ok:
          raise OSError(f"CH347 I2C block read data failed, ack={ack.value}")

        return bytes(rbuf)
      except OSError:
        if attempt == 2:
          raise
        time.sleep(0.001)

  def open(self) -> None:
    self.fd = self.lib.CH347OpenDevice(self.dev_path.encode())
    if self.fd < 0:
      raise OSError(f"failed to open {self.dev_path}")

    if not self.lib.CH34xSetTimeout(self.fd, 2000, 2000):
      raise OSError("CH34xSetTimeout failed")

    if not self.lib.CH347I2C_Set(self.fd, 0x01):  # 100kHz (more stable)
      raise OSError("CH347I2C_Set failed")

    self.lib.CH347I2C_SetIgnoreNack(self.fd, 1)
    self.lib.CH347I2C_SetStretch(self.fd, True)

    time.sleep(0.02)

    for _ in range(8):
      for addr in CH347_LSM6_ADDRS:
        try:
          who = self._stream_read([addr << 1, CH347_WHO_AM_I_REG], 1)[0]
          if who in CH347_WHO_AM_I_IDS:
            self.addr = addr
            self.source = log.SensorEventData.SensorSource.lsm6ds3trc if who == 0x6A else log.SensorEventData.SensorSource.lsm6ds3
            cloudlog.info(f"CH347 LSM6DS3 detected at addr=0x{addr:02X}, who=0x{who:02X}")
            return
        except Exception:
          continue
      time.sleep(0.02)

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
      return None
    cloudlog.info(f"Found CH347 devices: {devs}, using {devs[0]}")
    dev = devs[0]

  lib_path = os.getenv("SENSORD_CH347_LIB", CH347_DEFAULT_LIB_PATH)

  if not os.path.exists(dev):
    cloudlog.error(f"CH347 device does not exist: {dev}")
    return None
  if not os.path.exists(lib_path):
    cloudlog.error(f"CH347 library does not exist: {lib_path}")
    return None

  cloudlog.info(f"CH347 backend: dev={dev}, lib={lib_path}")
  return dev, lib_path


def ch347_loop(dev_path: str, lib_path: str, event: threading.Event) -> None:
  sensor = CH347LSM6(dev_path, lib_path)
  pm = messaging.PubMaster(["accelerometer", "gyroscope", "temperatureSensor"])
  rk = Ratekeeper(104, print_delay_threshold=None)  # 104Hz matches sensor config
  temp_ticks = 0
  last_log_t = 0.0
  last_acc_v = None
  last_gyro_v = None

  # IMU axis rotation config (0, 90, 180, 270)
  try:
    from openpilot.common.params import Params
    params = Params()
    rotation = params.get_int("CH347ImuRotation")
  except Exception:
    try:
      rotation = int(os.getenv("SENSORD_CH347_ROTATION", "0"))
    except Exception:
      rotation = 0

  if rotation == 90 or rotation == 1:
    map_axes = lambda x, y, z: [x, y, z]
  elif rotation == 180 or rotation == 2:
    map_axes = lambda x, y, z: [-y, x, z]
  elif rotation == 270 or rotation == 3:
    map_axes = lambda x, y, z: [-x, -y, z]
  else:
    map_axes = lambda x, y, z: [y, -x, z]

  accel_scale = 9.81 * 2.0 / (1 << 15)
  gyro_scale = (8.75 / 1000.0) * (math.pi / 180.0)

  try:
    sensor.open()
    sensor.init_sensor()
    cloudlog.info(f"Using CH347 backend: dev={dev_path}, rotation={rotation}")

    consecutive_errors = 0
    last_error_log_t = 0.0
    is_status_read = False

    while not event.is_set():
      try:
        is_status_read = True
        status = sensor.read_u8(0x1E)
        is_status_read = False

        # accelerometer (if data ready)
        if status & 0x01:
          b = sensor.read_block(0x28, 6)
          x = Sensor.parse_16bit(b[0], b[1])
          y = Sensor.parse_16bit(b[2], b[3])
          z = Sensor.parse_16bit(b[4], b[5])
          last_acc_v = map_axes(x * accel_scale, y * accel_scale, z * accel_scale)

          msg = messaging.new_message("accelerometer", valid=True)
          msg.accelerometer.source = sensor.source
          msg.accelerometer.timestamp = msg.logMonoTime
          acc = msg.accelerometer.init("acceleration")
          acc.v = last_acc_v
          pm.send("accelerometer", msg)

        # gyroscope (if data ready)
        if status & 0x02:
          b = sensor.read_block(0x22, 6)
          x = Sensor.parse_16bit(b[0], b[1])
          y = Sensor.parse_16bit(b[2], b[3])
          z = Sensor.parse_16bit(b[4], b[5])
          last_gyro_v = map_axes(x * gyro_scale, y * gyro_scale, z * gyro_scale)
          msg = messaging.new_message("gyroscope", valid=True)
          msg.gyroscope.source = sensor.source
          msg.gyroscope.timestamp = msg.logMonoTime
          gyro = msg.gyroscope.init("gyroUncalibrated")
          gyro.v = last_gyro_v
          pm.send("gyroscope", msg)

        temp_ticks += 1

        current_t = time.monotonic()
        if current_t - last_log_t >= 10.0:
          if last_acc_v:
            cloudlog.warning(f"[CH347_ACCEL] X={last_acc_v[0]:6.3f}, Y={last_acc_v[1]:6.3f}, Z={last_acc_v[2]:6.3f} m/s²")
          if last_gyro_v:
            cloudlog.warning(f"[CH347_GYRO] X={last_gyro_v[0]:6.3f}, Y={last_gyro_v[1]:6.3f}, Z={last_gyro_v[2]:6.3f} rad/s")
          last_log_t = current_t

        if temp_ticks >= 10:
          temp_ticks = 0
          tb = sensor.read_block(0x20, 2)
          raw_t = Sensor.parse_16bit(tb[0], tb[1])
          tscale = 16.0 if sensor.source == log.SensorEventData.SensorSource.lsm6ds3 else 256.0

          msg = messaging.new_message("temperatureSensor", valid=True)
          msg.temperatureSensor.source = sensor.source
          msg.temperatureSensor.timestamp = msg.logMonoTime
          msg.temperatureSensor.temperature = 25 + (raw_t / tscale)
          pm.send("temperatureSensor", msg)

        consecutive_errors = 0
        rk.keep_time()

      except OSError as e:
        consecutive_errors += 1
        if consecutive_errors > 100:
          cloudlog.warning(f"CH347 I2C persistent error, attempting reconnect... (consecutive={consecutive_errors}): {e}")
          try:
            sensor.shutdown()
            time.sleep(0.5)
            sensor.open()
            sensor.init_sensor()
            consecutive_errors = 0
            is_status_read = False
            cloudlog.info("CH347 reconnected successfully!")
            continue
          except Exception as re_err:
            cloudlog.error(f"CH347 reconnection failed: {re_err}")
            if consecutive_errors > 300:
              raise OSError(f"CH347 persistent I2C error after multiple reconnect attempts (consecutive={consecutive_errors}): {e}") from e

        now = time.monotonic()
        if now - last_error_log_t > 1.0:
          if is_status_read:
            cloudlog.warning(f"CH347 I2C status read error ignored:\n{e}")
          else:
            cloudlog.warning(f"CH347 I2C error ignored (consecutive={consecutive_errors}): {e}")
          last_error_log_t = now
        is_status_read = False
        time.sleep(0.01)
  finally:
    sensor.shutdown()


# *** Local I2C (smbus) backend ***

def detect_imu_i2c_bus() -> tuple[int, int]:
  """Returns (bus, address) of the first LSM6DS3 found."""
  bus_override = os.getenv("SENSORD_I2C_BUS")
  addr_override = os.getenv("SENSORD_I2C_ADDR")
  if bus_override is not None and addr_override is not None:
    return int(bus_override), int(addr_override, 0)

  candidates = []
  for node in sorted(glob.glob("/dev/i2c-*"), key=lambda p: int(p.rsplit("-", 1)[1])):
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
      i2c = SMBus(bus)
    except OSError as e:
      cloudlog.warning(f"Failed to open I2C bus {bus}: {e}")
      continue

    try:
      for addr in LSM6DS3_I2C_ADDRS:
        try:
          chip_id = i2c.read_byte_data(addr, LSM6DS3_WHO_AM_I_REG)
        except OSError:
          # expected while scanning buses the IMU is not on; don't spam the log
          cloudlog.debug(f"no LSM6DS3 on bus {bus} at addr 0x{addr:02X}")
          continue

        if chip_id in LSM6DS3_WHO_AM_I_IDS:
          cloudlog.info(f"Detected LSM6DS3 on /dev/i2c-{bus}, addr=0x{addr:02X}, chip=0x{chip_id:02X}")
          return bus, addr
    finally:
      i2c.close()

  cloudlog.warning(f"Unable to auto-detect LSM6DS3, falling back to /dev/i2c-{DEFAULT_I2C_BUS_IMU} @ 0x{LSM6DS3_I2C_ADDRS[0]:02X}")
  return DEFAULT_I2C_BUS_IMU, LSM6DS3_I2C_ADDRS[0]


def has_imu_interrupt() -> bool:
  try:
    fd = gpiochip_get_ro_value_fd("sensord", 0, 84)
    os.close(fd)
    return True
  except Exception:
    cloudlog.warning("IMU GPIO interrupt unavailable, using polling fallback for accel/gyro")
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


def reinit_sensor(sensor: Sensor) -> None:
  """Offline recovery: re-open the I2C bus and re-initialize the sensor.

  Mirrors the reconnect path of the CH347 backend and the rebind logic of the
  reference ubuntu347t sensord (sensor_open_retry + reinit on I2C errors).
  """
  try:
    sensor.shutdown()
  except Exception:
    pass
  try:
    sensor.bus.close()
  except Exception:
    pass
  sensor.bus = SMBus(sensor.bus_num)
  sensor.reset()
  sensor.init()


# ~1s of consecutive read failures at the 100Hz service rate before a re-init
# is attempted; ~30 failed re-inits (with 1s backoff) before giving up.
SENSORD_OFFLINE_ERROR_THRESHOLD = 100
SENSORD_OFFLINE_REINIT_LIMIT = 30
# identical raw samples for ~1s at the 104Hz ODR means the sensor/bus is stuck
SENSORD_FROZEN_FRAME_LIMIT = 104


def polling_loop(sensor: Sensor, service: str, event: threading.Event, needs_timestamp: bool = False) -> None:
  pm = messaging.PubMaster([service])
  rk = Ratekeeper(SERVICE_LIST[service].frequency, print_delay_threshold=None)
  last_log_t = 0.0
  last_error_log_t = 0.0
  consecutive_errors = 0
  reinit_attempts = 0
  had_valid_data = False
  last_sample = None
  frozen_frames = 0
  while not event.is_set():
    try:
      evt = sensor.get_event(time.time_ns()) if needs_timestamp else sensor.get_event()
      if not sensor.is_data_valid():
        rk.keep_time()
        continue

      # frozen-data detection: a healthy IMU never repeats the exact same raw
      # sample for a full second; if it does, the bus or sensor is stuck.
      sample = None
      if service == "accelerometer":
        sample = tuple(evt.acceleration.v)
      elif service == "gyroscope":
        sample = tuple(evt.gyroUncalibrated.v)
      if sample is not None and sample == last_sample:
        frozen_frames += 1
        if frozen_frames >= SENSORD_FROZEN_FRAME_LIMIT:
          raise Sensor.SensorException(f"{service} data frozen for {frozen_frames} samples (stuck bus?)")
      else:
        frozen_frames = 0
        last_sample = sample

      msg = messaging.new_message(service, valid=True)
      setattr(msg, service, evt)
      pm.send(service, msg)

      had_valid_data = True
      consecutive_errors = 0
      reinit_attempts = 0

      # print a sample every 10s (mirrors the CH347 backend logging)
      current_t = time.monotonic()
      if current_t - last_log_t >= 10.0:
        if service == "accelerometer":
          v = evt.acceleration.v
          cloudlog.warning(f"[SENSORD_ACCEL] X={v[0]:6.3f}, Y={v[1]:6.3f}, Z={v[2]:6.3f} m/s²")
        elif service == "gyroscope":
          v = evt.gyroUncalibrated.v
          cloudlog.warning(f"[SENSORD_GYRO] X={v[0]:6.3f}, Y={v[1]:6.3f}, Z={v[2]:6.3f} rad/s")
        elif service == "temperatureSensor":
          cloudlog.warning(f"[SENSORD_TEMP] {evt.temperature:.1f} °C")
        last_log_t = current_t
    except Sensor.DataNotReady:
      pass  # normal in polling mode: sample not ready yet
    except Exception as e:
      consecutive_errors += 1
      now = time.monotonic()
      if now - last_error_log_t > 1.0:
        cloudlog.warning(f"[SENSORD] {service} read error (consecutive={consecutive_errors}): {e}")
        last_error_log_t = now

      if consecutive_errors >= SENSORD_OFFLINE_ERROR_THRESHOLD:
        # ~1s of persistent failures: try in-process recovery first
        reinit_attempts += 1
        cloudlog.warning(f"[SENSORD] {service} offline, re-initializing sensor (attempt {reinit_attempts}): {e}")
        try:
          reinit_sensor(sensor)
          consecutive_errors = 0
          frozen_frames = 0
          last_sample = None
          cloudlog.warning(f"[SENSORD] {service} sensor re-initialized successfully")
        except Exception as re_err:
          cloudlog.error(f"[SENSORD] {service} re-init failed (attempt {reinit_attempts}): {re_err}")
          # IMU was working before but we can't recover in-process: exit so
          # manager restarts sensord with a full bus re-scan. This also handles
          # an unplug/replug onto a different I2C bus.
          if had_valid_data and reinit_attempts >= SENSORD_OFFLINE_REINIT_LIMIT:
            cloudlog.error(f"[SENSORD] {service} unrecoverable after {reinit_attempts} re-init attempts, "
                           "exiting for manager restart")
            os._exit(1)
          time.sleep(1.0)  # backoff, don't hammer a dead bus
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
      if "not detected" in str(e):
        cloudlog.error(f"CH347 sensor detection failed: {e}")
      elif "failed to open" in str(e):
        cloudlog.error(f"CH347 device not found or not accessible: {e}")
      else:
        cloudlog.exception(f"CH347 I2C error: {e}")
      cloudlog.info("Falling back to smbus path")
    except Exception:
      cloudlog.exception("CH347 backend failed with unexpected error, falling back to smbus path")

  i2c_bus_imu, imu_addr = detect_imu_i2c_bus()
  interrupt_available = has_imu_interrupt()

  sensors_cfg = [
    (LSM6DS3_Accel(i2c_bus_imu, imu_addr), "accelerometer", interrupt_available),
    (LSM6DS3_Gyro(i2c_bus_imu, imu_addr), "gyroscope", interrupt_available),
    (LSM6DS3_Temp(i2c_bus_imu, imu_addr), "temperatureSensor", False),
  ]

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
