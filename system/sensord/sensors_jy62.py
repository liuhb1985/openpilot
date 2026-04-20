#!/usr/bin/env python3
"""
JY62 UART IMU Sensor Main Script
Follows the official sensord pattern but for UART-based sensor
"""

import time
import threading
import sys
import serial
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import config_realtime_process, Ratekeeper
from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from cereal.services import SERVICE_LIST
from cereal import log
import cereal.messaging as messaging

# JY62 协议常量
FRAME_HEADER_1 = 0x55
FRAME_HEADER_2_ACCEL_ANGLE = 0x53  # 加速度和角度数据
FRAME_HEADER_2_TIME_ACCEL = 0x51   # 时间和加速度数据
FRAME_HEADER_2_ANGLE = 0x52        # 角度数据


class JY62_UART(Sensor):
    """JY62 IMU sensor via UART"""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int | None = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.buffer = bytearray()
        self.source = log.SensorEventData.SensorSource.android
        self.start_ts = 0.
        self.bus = None  # 不使用I2C

        # 如果用户传入 None 或者 0，将尝试自动扫描常见波特率
        if self.baudrate is None or self.baudrate == 0:
            self.baudrate = self._auto_baud()
            cloudlog.info(f"Auto-detected baudrate: {self.baudrate}")

        # 直接初始化串口，不调用parent的初始化
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            cloudlog.info(f"JY62 UART initialized on {self.port} at {self.baudrate} baud")
        except Exception as e:
            cloudlog.error(f"Failed to open JY62 UART: {e}")
            raise

    @property
    def device_address(self) -> int:
        # UART 没有设备地址
        return 0xFF

    def reset(self):
        """重置传感器"""
        try:
            if self.ser:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            time.sleep(0.1)
            cloudlog.info("JY62 reset complete")
        except Exception as e:
            cloudlog.error(f"JY62 reset failed: {e}")

    def _auto_baud(self) -> int:
        """尝试多个常见波特率并返回第一个能解析出数据的速率"""
        candidates = [115200, 57600, 38400, 19200, 9600]
        for br in candidates:
            try:
                tmp = serial.Serial(self.port, br, timeout=0.1)
                tmp.reset_input_buffer()
                # 读取一点数据尝试发现帧头
                data = tmp.read(20)
                tmp.close()
                if data and 0x55 in data:
                    return br
            except Exception:
                pass
        raise RuntimeError("unable to auto-detect baud rate")

    def init(self):
        """初始化传感器"""
        try:
            if not self.ser:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.ser.reset_input_buffer()

            # 停止连续输出
            self.ser.write(bytes([0xFF, 0xAA, 0x02, 0x00, 0x00]))
            time.sleep(0.1)

            # 加速度校准（传感器水平放置时）
            self.ser.write(bytes([0xFF, 0xAA, 0x01, 0x01]))
            time.sleep(1.0)

            # 保存校准参数
            self.ser.write(bytes([0xFF, 0xAA, 0x00, 0x00]))
            time.sleep(0.1)

            # 设置频率为10Hz (0x27 0x01)
            self.ser.write(bytes([0xFF, 0xAA, 0x27, 0x01]))
            time.sleep(0.1)

            # 启动连续输出
            self.ser.write(bytes([0xFF, 0xAA, 0x02, 0x01, 0x00]))
            time.sleep(0.1)

            cloudlog.info("JY62 init complete")
        except Exception as e:
            cloudlog.error(f"JY62 init failed: {e}")
            raise

    @staticmethod
    def _parse_int16(low: int, high: int) -> int:
        """Parse little-endian 16-bit signed integer"""
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def _read_frame(self) -> dict | None:
        """从串口读一个字节并尝试解析完整帧

        JY62 所有帧都是11字节：
        [0x55][Type][Payload(9 bytes)]

        类型:
        0x51: 加速度数据 - [0x55][0x51][TimeL][TimeH][AxL][AxH][AyL][AyH][AzL][AzH][CheckSum]
        0x52: 角速度 - [0x55][0x52][WxL][WxH][WyL][WyH][WzL][WzH][...][CheckSum]
        0x53: 温度/指南针 - [0x55][0x53][...]
        """
        if not self.ser:
            return None

        try:
            data = self.ser.read(1)
            if not data:
                return None

            self.buffer.extend(data)

            # 查找 0x55 帧头
            while len(self.buffer) >= 11:  # 所有帧都是11字节
                header_idx = self.buffer.find(0x55)
                if header_idx == -1:
                    self.buffer.clear()
                    return None

                # 移除帧头前的所有数据
                if header_idx > 0:
                    self.buffer = self.buffer[header_idx:]
                    header_idx = 0

                if len(self.buffer) < 2:
                    break

                frame_type = self.buffer[1]

                if frame_type == 0x51:
                    # 0x55 0x51: 加速度数据 (11 bytes)
                    # [0x55][0x51][AxL][AxH][AyL][AyH][AzL][AzH][TL][TH][CheckSum]
                    if len(self.buffer) < 11:
                        break

                    # 提取加速度: 索引[2:3], [4:5], [6:7]
                    ax_raw = self._parse_int16(self.buffer[2], self.buffer[3])
                    ay_raw = self._parse_int16(self.buffer[4], self.buffer[5])
                    az_raw = self._parse_int16(self.buffer[6], self.buffer[7])

                    self.buffer = self.buffer[11:]
                    return {'ax': ax_raw, 'ay': ay_raw, 'az': az_raw}

                elif frame_type == 0x52:
                    # 0x55 0x52: 角速度数据 (11 bytes)
                    # [0x55][0x52][WxL][WxH][WyL][WyH][WzL][WzH][...][CheckSum]
                    if len(self.buffer) < 11:
                        break

                    wx_raw = self._parse_int16(self.buffer[2], self.buffer[3])
                    wy_raw = self._parse_int16(self.buffer[4], self.buffer[5])
                    wz_raw = self._parse_int16(self.buffer[6], self.buffer[7])

                    self.buffer = self.buffer[11:]
                    return {'wx': wx_raw, 'wy': wy_raw, 'wz': wz_raw}

                elif frame_type == 0x53:
                    # 0x55 0x53: 温度/指南针 (11 bytes)
                    # 暂时跳过，不需要处理
                    if len(self.buffer) < 11:
                        break
                    self.buffer = self.buffer[11:]
                    # 跳过这一帧，继续查找下一帧
                    continue

                else:
                    # 跳过未知帧类型，往前移1个字节继续查找
                    self.buffer = self.buffer[1:]

        except Exception as e:
            cloudlog.error(f"JY62 frame parse error: {e}")

        return None

    def get_accel_event(self, ts: int | None = None) -> log.SensorEventData:
        """获取加速度传感器事件"""
        # 持续读取直到得到加速度数据（0x51帧）
        max_attempts = 50
        attempts = 0

        while attempts < max_attempts:
            frame = self._read_frame()
            if frame and 'ax' in frame and 'wx' not in frame:
                break
            attempts += 1
            time.sleep(0.001)

        if not frame or 'ax' not in frame:
            raise self.DataNotReady

        # 将原始值转换为 m/s²
        # JY62: ±16g, 16-bit signed, 所以单位 = 16 * 9.81 / 32768
        SCALE = 16.0 * 9.80665 / 32768.0
        ax = frame['ax'] * SCALE
        ay = frame['ay'] * SCALE
        az = frame['az'] * SCALE

        event = log.SensorEventData.new_message()
        event.timestamp = ts if ts is not None else time.monotonic_ns()
        event.version = 1
        event.sensor = 1  # SENSOR_ACCELEROMETER
        event.type = 1    # SENSOR_TYPE_ACCELEROMETER
        event.source = self.source

        a = event.init('acceleration')
        a.v = [ax, ay, az]
        a.status = 1

        return event

    def get_gyro_event(self, ts: int | None = None) -> log.SensorEventData:
        """获取陀螺仪传感器事件"""
        # 持续读取直到得到陀螺仪数据（0x52帧）
        max_attempts = 50
        attempts = 0

        while attempts < max_attempts:
            frame = self._read_frame()
            if frame and 'wx' in frame:
                break
            attempts += 1
            time.sleep(0.001)

        if not frame or 'wx' not in frame:
            raise self.DataNotReady

        # 将原始值转换为 rad/s
        # JY62: ±2000°/s, 16-bit signed, 所以单位 = 2000° / 32768 / (180/π)
        GYRO_SCALE = (2000.0 / 32768.0) * (3.14159265359 / 180.0)  # rad/s
        wx = frame['wx'] * GYRO_SCALE
        wy = frame['wy'] * GYRO_SCALE
        wz = frame['wz'] * GYRO_SCALE

        event = log.SensorEventData.new_message()
        event.timestamp = ts if ts is not None else time.monotonic_ns()
        event.version = 2
        event.sensor = 5  # SENSOR_GYRO_UNCALIBRATED
        event.type = 16   # SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
        event.source = self.source

        g = event.init('gyroUncalibrated')
        g.v = [wx, wy, wz]
        g.status = 1

        return event

    def get_event(self, ts: int | None = None) -> log.SensorEventData:
        """默认返回加速度事件"""
        return self.get_accel_event(ts)

    def shutdown(self) -> None:
        """关闭传感器"""
        try:
            if self.ser:
                self.ser.write(bytes([0xFF, 0xAA, 0x02, 0x00, 0x00]))  # 停止输出
                time.sleep(0.1)
                self.ser.close()
                self.ser = None
            cloudlog.info("JY62 shutdown complete")
        except Exception as e:
            cloudlog.error(f"JY62 shutdown error: {e}")

    def is_data_valid(self) -> bool:
        """检查数据有效性"""
        if self.start_ts == 0:
            self.start_ts = time.monotonic()
        return (time.monotonic() - self.start_ts) > 0.5


def polling_loop(sensor: JY62_UART, service: str, event: threading.Event) -> None:
    """JY62 polling loop - reads sensor data and publishes"""
    if service == "accelerometer":
        pm = messaging.PubMaster(['accelerometer', 'gyroscope'])
    else:
        pm = messaging.PubMaster([service])

    rk = Ratekeeper(SERVICE_LIST[service].frequency, print_delay_threshold=None)

    cloudlog.info("JY62 polling loop started")
    # print("JY62 polling loop started")
    # sys.stdout.flush()

    # only print debug info at ~1Hz, not every iteration
    last_print = time.monotonic()
    while not event.is_set():
        try:
            # 发布加速度
            if service == "accelerometer":
                try:
                    evt = sensor.get_accel_event()
                    if not sensor.is_data_valid():
                        rk.keep_time()
                        continue

                    msg = messaging.new_message(service, valid=True)
                    setattr(msg, service, evt)
                    pm.send(service, msg)

                    # debug print once per second
                    now = time.monotonic()
                    if now - last_print >= 1.0:
                        # accel = evt.acceleration
                        # print(f"加速度 XYZ: X={accel.v[0]:.3f} Y={accel.v[1]:.3f} Z={accel.v[2]:.3f}")
                        # sys.stdout.flush()
                        last_print = now
                except JY62_UART.DataNotReady:
                    pass

                # 同时尝试发布陀螺仪（低优先级）
                try:
                    evt_gyro = sensor.get_gyro_event()
                    if sensor.is_data_valid():
                        msg_gyro = messaging.new_message('gyroscope', valid=True)
                        msg_gyro.gyroscope = evt_gyro
                        pm.send('gyroscope', msg_gyro)

                        gyro_print_time = time.monotonic()
                        if gyro_print_time - last_print >= 1.0:
                            # gyro = evt_gyro.gyroUncalibrated
                            # print(f"陀螺仪 XYZ: X={gyro.v[0]:.3f} Y={gyro.v[1]:.3f} Z={gyro.v[2]:.3f}")
                            # sys.stdout.flush()
                            last_print = gyro_print_time
                except JY62_UART.DataNotReady:
                    pass
                except Exception as e:
                    cloudlog.error(f"陀螺仪错误: {e}")
            else:
                evt = sensor.get_event()
                if not sensor.is_data_valid():
                    rk.keep_time()
                    continue

                msg = messaging.new_message(service, valid=True)
                setattr(msg, service, evt)
                pm.send(service, msg)

        except JY62_UART.DataNotReady:
            pass
        except Exception as e:
            cloudlog.exception(f"Error in {service} polling loop: {e}")

        rk.keep_time()


def main() -> None:
    """Main entry point"""
    config_realtime_process([1, ], 1)

    cloudlog.info("sensord_jy62 main() started")
    # print("sensord_jy62 main() started")
    # sys.stdout.flush()

    try:
        sensor = JY62_UART(port="/dev/ttyUSB0", baudrate=115200)
        # print("JY62 sensor created successfully")
        # sys.stdout.flush()

        # Reset sensor
        try:
            sensor.reset()
        except Exception as e:
            cloudlog.exception(f"Error resetting sensor: {e}")

        # Initialize sensor
        try:
            sensor.init()
            # print("JY62 sensor initialized successfully")
            # sys.stdout.flush()
        except Exception as e:
            cloudlog.exception(f"Error initializing sensor: {e}")
            print(f"Error initializing sensor: {e}")
            sys.stdout.flush()
            return

        # Create exit event and polling thread
        exit_event = threading.Event()
        polling_thread = threading.Thread(
            target=polling_loop,
            args=(sensor, "accelerometer", exit_event),
            daemon=True
        )

        try:
            polling_thread.start()
            # print("JY62 polling thread started")
            # sys.stdout.flush()

            # Keep main thread alive
            while polling_thread.is_alive():
                time.sleep(1)

        except KeyboardInterrupt:
            cloudlog.info("Keyboard interrupt received")

        finally:
            exit_event.set()
            if polling_thread.is_alive():
                polling_thread.join(timeout=5)

            try:
                sensor.shutdown()
                # print("JY62 sensor shutdown complete")
                # sys.stdout.flush()
            except Exception as e:
                cloudlog.exception(f"Error shutting down sensor: {e}")

    except Exception as e:
        cloudlog.exception(f"Fatal error in JY62 sensord: {e}")
        # print(f"Fatal error: {e}")
        # sys.stdout.flush()
        raise


if __name__ == "__main__":
    main()

