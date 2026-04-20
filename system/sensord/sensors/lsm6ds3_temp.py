import time

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor

# https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf
class LSM6DS3_Temp(Sensor):
  LSM6DS3_TEMP_I2C_ADDRS = (0x6A, 0x6B)
  LSM6DS3_TEMP_WHO_AM_I_REG = 0x0F
  LSM6DS3_TEMP_WHO_AM_I_IDS = (0x69, 0x6A)

  @property
  def device_address(self) -> int:
    if not hasattr(self, "_device_address"):
      self._device_address, self._chip_id = self.detect_i2c_device(
        self.LSM6DS3_TEMP_I2C_ADDRS,
        self.LSM6DS3_TEMP_WHO_AM_I_REG,
        self.LSM6DS3_TEMP_WHO_AM_I_IDS,
      )
    return self._device_address

  def _read_temperature(self) -> float:
    scale = 16.0 if self.source == log.SensorEventData.SensorSource.lsm6ds3 else 256.0
    data = self.read(0x20, 2)
    return 25 + (self.parse_16bit(data[0], data[1]) / scale)

  def init(self):
    chip_id = self.verify_chip_id(
      self.LSM6DS3_TEMP_WHO_AM_I_REG,
      list(self.LSM6DS3_TEMP_WHO_AM_I_IDS),
    )
    if chip_id == 0x6A:
      self.source = log.SensorEventData.SensorSource.lsm6ds3trc
    else:
      self.source = log.SensorEventData.SensorSource.lsm6ds3

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    event = log.SensorEventData.new_message()
    event.version = 1
    event.timestamp = int(time.monotonic() * 1e9)
    event.source = self.source
    event.temperature = self._read_temperature()
    return event

  def shutdown(self) -> None:
    pass
