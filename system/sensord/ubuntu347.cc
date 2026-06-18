#include <cstdint>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/usbdevice_fs.h>
#include <dirent.h>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <array>

// CH347 USB VID/PID
constexpr const char* CH347_USB_VID = "1a86";
constexpr const char* CH347_USB_PID = "55db";
constexpr uint8_t CH347_WHO_AM_I_REG = 0x0F;
constexpr uint8_t CH347_LSM6_ADDRS[] = {0x6B, 0x6A};
constexpr uint8_t CH347_WHO_AM_I_IDS[] = {0x69, 0x6A};

// 冻结检测阈值
constexpr int FREEZE_EXIT_THRESHOLD = 8;
constexpr int FREEZE_MAX_REBOUNDS = 2;
constexpr int FROZEN_STALE_THRESHOLD = 4;

static std::atomic<bool> g_running(true);

void signal_handler(int signo) {
  g_running.store(false);
}

struct LinuxI2CLSM6 {
  int bus_num = -1;
  int fd = -1;
  uint8_t addr = 0;
  bool source_is_lsm6ds3trc = false;

  bool _i2c_rdwr(const uint8_t* write_bytes, size_t write_len, uint8_t* read_buf, size_t read_len) {
    struct i2c_msg msgs[2];
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = write_len;
    msgs[0].buf = const_cast<__u8*>(write_bytes);

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = read_len;
    msgs[1].buf = read_buf;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;

    return ioctl(fd, I2C_RDWR, &ioctl_data) == 0;
  }

  bool _i2c_write(const uint8_t* write_bytes, size_t write_len) {
    struct i2c_msg msgs[1];
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = write_len;
    msgs[0].buf = const_cast<__u8*>(write_bytes);

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 1;

    return ioctl(fd, I2C_RDWR, &ioctl_data) == 0;
  }

  uint8_t read_u8(uint8_t reg) {
    uint8_t rbuf[1];
    _i2c_rdwr(&reg, 1, rbuf, 1);
    return rbuf[0];
  }

  bool write_u8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return _i2c_write(buf, 2);
  }

  bool open_dev() {
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", bus_num);
    fd = ::open(dev_path, O_RDWR);
    if (fd < 0) return false;

    for (int attempt = 0; attempt < 8; attempt++) {
      for (size_t i = 0; i < sizeof(CH347_LSM6_ADDRS); i++) {
        addr = CH347_LSM6_ADDRS[i];
        uint8_t who = read_u8(CH347_WHO_AM_I_REG);
        for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
          if (who == CH347_WHO_AM_I_IDS[j]) {
            source_is_lsm6ds3trc = (who == 0x6A);
            return true;
          }
        }
      }
      usleep(20000);
    }
    close_dev();
    return false;
  }

  bool init_sensor() {
    if (!write_u8(0x12, 0x01)) return false;  // CTRL3_C: SW_RESET
    usleep(100000);
    if (!write_u8(0x12, 0x44)) return false;  // IF_INC=1, BDU=1
    if (!write_u8(0x10, 0x40)) return false;  // Accel: 104Hz ODR, +-2g
    if (!write_u8(0x11, 0x40)) return false;  // Gyro: 104Hz ODR, +-245dps
    return true;
  }

  bool read_block(uint8_t start_reg, uint8_t* buf, size_t len) {
    return _i2c_rdwr(&start_reg, 1, buf, len);
  }

  void close_dev() {
    if (fd >= 0) {
      write_u8(0x10, 0x00);
      write_u8(0x11, 0x00);
      ::close(fd);
      fd = -1;
    }
  }
};

int find_ch347_i2c_bus() {
  DIR* dir = opendir("/sys/bus/i2c/devices");
  if (dir) {
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strncmp(entry->d_name, "i2c-", 4) != 0) continue;
      char bus_path[256], name_path[256];
      snprintf(bus_path, sizeof(bus_path), "/sys/bus/i2c/devices/%s/name", entry->d_name);
      std::ifstream name_file(bus_path);
      std::string bus_name;
      if (name_file.is_open()) std::getline(name_file, bus_name);

      int bus_num = atoi(entry->d_name + 4);
      if (bus_name.find("ch347") != std::string::npos) {
        closedir(dir);
        return bus_num;
      }
    }
    closedir(dir);
  }

  // 扫描所有 I2C 总线
  for (int bus = 0; bus < 10; bus++) {
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", bus);
    int fd = ::open(dev_path, O_RDWR);
    if (fd < 0) continue;

    for (size_t i = 0; i < sizeof(CH347_LSM6_ADDRS); i++) {
      uint8_t addr = CH347_LSM6_ADDRS[i];
      if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0) continue;

      uint8_t wbuf = CH347_WHO_AM_I_REG;
      uint8_t rbuf;
      struct i2c_msg msgs[2];
      msgs[0].addr = addr;
      msgs[0].flags = 0;
      msgs[0].len = 1;
      msgs[0].buf = &wbuf;
      msgs[1].addr = addr;
      msgs[1].flags = I2C_M_RD;
      msgs[1].len = 1;
      msgs[1].buf = &rbuf;

      struct i2c_rdwr_ioctl_data ioctl_data;
      ioctl_data.msgs = msgs;
      ioctl_data.nmsgs = 2;

      if (ioctl(fd, I2C_RDWR, &ioctl_data) == 0) {
        for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
          if (rbuf == CH347_WHO_AM_I_IDS[j]) {
            ::close(fd);
            return bus;
          }
        }
      }
    }
    ::close(fd);
  }
  return -1;
}

std::string find_ch347_usb_device() {
  DIR* dir = opendir("/sys/bus/usb/drivers/usb");
  if (!dir) return "";
  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_name[0] == '.') continue;
    char vid_path[256], pid_path[256];
    snprintf(vid_path, sizeof(vid_path), "/sys/bus/usb/drivers/usb/%s/idVendor", entry->d_name);
    snprintf(pid_path, sizeof(pid_path), "/sys/bus/usb/drivers/usb/%s/idProduct", entry->d_name);

    std::ifstream vid_f(vid_path), pid_f(pid_path);
    std::string vid, pid;
    if (vid_f.is_open()) std::getline(vid_f, vid);
    if (pid_f.is_open()) std::getline(pid_f, pid);

    if (vid == CH347_USB_VID && pid == CH347_USB_PID) {
      closedir(dir);
      return entry->d_name;
    }
  }
  closedir(dir);
  return "";
}

bool usb_rebind_ch347(const std::string& usb_dev) {
  char unbind_path[256], bind_path[256];
  snprintf(unbind_path, sizeof(unbind_path), "/sys/bus/usb/drivers/usb/unbind");
  snprintf(bind_path, sizeof(bind_path), "/sys/bus/usb/drivers/usb/bind");

  // unbind
  {
    std::ofstream f(unbind_path);
    if (f.is_open()) f << usb_dev;
  }
  usleep(100000);

  // bind
  {
    std::ofstream f(bind_path);
    if (f.is_open()) f << usb_dev;
  }
  usleep(500000);

  return find_ch347_i2c_bus() >= 0;
}

struct I2CState {
  LinuxI2CLSM6 sensor;
  uint8_t last_acc_raw[6] = {0};
  uint8_t last_gyro_raw[6] = {0};
  bool has_acc_raw = false;
  bool has_gyro_raw = false;
  float last_acc_v[3] = {0};
  float last_gyro_v[3] = {0};
  int consecutive_errors = 0;
  int freeze_counter = 0;
  int rebind_count = 0;
};

void i2c_loop(int bus_num) {
  I2CState state;
  state.sensor.bus_num = bus_num;

  if (!state.sensor.open_dev()) {
    fprintf(stderr, "Failed to open sensor on /dev/i2c-%d\n", bus_num);
    return;
  }
  if (!state.sensor.init_sensor()) {
    fprintf(stderr, "Failed to initialize sensor\n");
    state.sensor.close_dev();
    return;
  }

  int temp_ticks = 0;
  auto last_log_t = std::chrono::steady_clock::now();
  auto stale_log_t = std::chrono::steady_clock::now();
  auto freeze_log_t = std::chrono::steady_clock::now();

  while (g_running.load()) {
    bool i2c_error = false;
    bool fresh_acc_valid = false;
    bool fresh_gyro_valid = false;
    float fresh_acc[3] = {0};
    float fresh_gyro[3] = {0};
    uint8_t fresh_acc_raw[6] = {0};
    uint8_t fresh_gyro_raw[6] = {0};

    // 1. STATUS_REG
    uint8_t status = 0;
    if (!state.sensor.read_u8(0x1E)) {
      i2c_error = true;
    } else {
      status = state.sensor.read_u8(0x1E);
    }

    // 2. Accelerometer
    if (status & 0x01) {
      if (state.sensor.read_block(0x28, fresh_acc_raw, 6)) {
        float x = (int16_t)((uint16_t)fresh_acc_raw[1] << 8 | fresh_acc_raw[0]);
        float y = (int16_t)((uint16_t)fresh_acc_raw[3] << 8 | fresh_acc_raw[2]);
        float z = (int16_t)((uint16_t)fresh_acc_raw[5] << 8 | fresh_acc_raw[4]);
        float scale = 9.81f * 2.0f / (1 << 15);
        fresh_acc[0] = y * scale;
        fresh_acc[1] = -x * scale;
        fresh_acc[2] = z * scale;
        fresh_acc_valid = true;
        memcpy(state.last_acc_v, fresh_acc, sizeof(fresh_acc));
      } else {
        i2c_error = true;
      }
    }

    // 3. Gyroscope
    if (status & 0x02) {
      if (state.sensor.read_block(0x22, fresh_gyro_raw, 6)) {
        float x = (int16_t)((uint16_t)fresh_gyro_raw[1] << 8 | fresh_gyro_raw[0]);
        float y = (int16_t)((uint16_t)fresh_gyro_raw[3] << 8 | fresh_gyro_raw[2]);
        float z = (int16_t)((uint16_t)fresh_gyro_raw[5] << 8 | fresh_gyro_raw[4]);
        float scale = (8.75f / 1000.0f) * (M_PI / 180.0f);
        fresh_gyro[0] = y * scale;
        fresh_gyro[1] = -x * scale;
        fresh_gyro[2] = z * scale;
        fresh_gyro_valid = true;
        memcpy(state.last_gyro_v, fresh_gyro, sizeof(fresh_gyro));
      } else {
        i2c_error = true;
      }
    }

    // 4. Error counting
    if (i2c_error) {
      state.consecutive_errors++;
    } else {
      state.consecutive_errors = 0;
    }

    // 5. Freeze detection
    bool acc_frozen = (fresh_acc_valid && state.has_acc_raw && memcmp(fresh_acc_raw, state.last_acc_raw, 6) == 0);
    bool gyro_frozen = (fresh_gyro_valid && state.has_gyro_raw && memcmp(fresh_gyro_raw, state.last_gyro_raw, 6) == 0);
    bool any_frozen = acc_frozen || gyro_frozen;
    bool acc_unfrozen = (fresh_acc_valid && state.has_acc_raw && memcmp(fresh_acc_raw, state.last_acc_raw, 6) != 0);
    bool gyro_unfrozen = (fresh_gyro_valid && state.has_gyro_raw && memcmp(fresh_gyro_raw, state.last_gyro_raw, 6) != 0);
    bool any_unfrozen = acc_unfrozen || gyro_unfrozen;

    if (any_frozen) {
      state.freeze_counter++;
      auto now = std::chrono::steady_clock::now();

      if (state.freeze_counter == 1 || state.freeze_counter == 7) {
        bool who_ok = false;
        uint8_t who = state.sensor.read_u8(0x0F);
        for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
          if (who == CH347_WHO_AM_I_IDS[j]) { who_ok = true; break; }
        }
        const char* fault = (i2c_error || !who_ok) ? "CH347T" : "LSM6DS3";
        fprintf(stderr, "[sensord] FREEZE fault=%s freeze=%d WHO=0x%02X\n",
                fault, state.freeze_counter, who);
      }

      auto diag_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - freeze_log_t).count();
      if (diag_elapsed >= 1) {
        fprintf(stderr, "[sensord] Data frozen! freeze_counter=%d acc_frozen=%d gyro_frozen=%d i2c_error=%d\n",
                state.freeze_counter, acc_frozen, gyro_frozen, i2c_error);
        freeze_log_t = now;
      }
    } else if (any_unfrozen) {
      if (state.freeze_counter > 0) {
        fprintf(stderr, "[sensord] Data unfrozen after %d frames\n", state.freeze_counter);
      }
      state.freeze_counter = 0;
    }

    // 6. Freeze recovery
    if (state.freeze_counter >= FREEZE_EXIT_THRESHOLD) {
      state.rebind_count++;
      if (state.rebind_count <= FREEZE_MAX_REBOUNDS) {
        fprintf(stderr, "[sensord] Attempting USB rebind (%d/%d)\n", state.rebind_count, FREEZE_MAX_REBOUNDS);
        std::string usb_dev = find_ch347_usb_device();
        if (!usb_dev.empty() && usb_rebind_ch347(usb_dev)) {
          int new_bus = find_ch347_i2c_bus();
          if (new_bus >= 0) {
            state.sensor.close_dev();
            state.sensor.bus_num = new_bus;
            if (state.sensor.open_dev() && state.sensor.init_sensor()) {
              state.freeze_counter = 0;
              state.consecutive_errors = 0;
              state.rebind_count = 0;
              state.has_acc_raw = false;
              state.has_gyro_raw = false;
              fprintf(stderr, "[sensord] Sensor recovered, new bus=/dev/i2c-%d\n", new_bus);
              continue;
            }
          }
        }
      }
      fprintf(stderr, "[sensord] Rebind exhausted, exiting\n");
      break;
    }

    // Save raw bytes
    if (fresh_acc_valid) {
      memcpy(state.last_acc_raw, fresh_acc_raw, 6);
      state.has_acc_raw = true;
    }
    if (fresh_gyro_valid) {
      memcpy(state.last_gyro_raw, fresh_gyro_raw, 6);
      state.has_gyro_raw = true;
    }

    // 7. Send (placeholder - in real code would send via cereal)
    bool serious_frozen = any_frozen && state.freeze_counter >= FROZEN_STALE_THRESHOLD;
    bool should_send_stale = i2c_error || (any_frozen && !serious_frozen);

    // 8. Logging
    auto now = std::chrono::steady_clock::now();
    auto log_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_t).count();
    if (log_elapsed >= 1) {
      if (serious_frozen) {
        uint8_t who = state.sensor.read_u8(0x0F);
        bool who_ok = false;
        for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
          if (who == CH347_WHO_AM_I_IDS[j]) { who_ok = true; break; }
        }
        const char* fault = (i2c_error || !who_ok) ? "CH347T" : "LSM6DS3";
        fprintf(stderr, "[sensord] FROZEN! fault=%s freeze=%d, waiting rebind...\n", fault, state.freeze_counter);
      } else if (state.has_acc_raw) {
        fprintf(stderr, "[sensord] ACC: [%9.4f, %9.4f, %9.4f] m/s2\n",
                state.last_acc_v[0], state.last_acc_v[1], state.last_acc_v[2]);
        fprintf(stderr, "[sensord] GYRO: [%9.5f, %9.5f, %9.5f] rad/s\n",
                state.last_gyro_v[0], state.last_gyro_v[1], state.last_gyro_v[2]);
      }
      last_log_t = now;
    }

    // Temperature
    temp_ticks++;
    if (temp_ticks >= 10) {
      temp_ticks = 0;
      uint8_t tb[2];
      if (state.sensor.read_block(0x20, tb, 2)) {
        int16_t raw_t = (int16_t)((uint16_t)tb[1] << 8 | tb[0]);
        float tscale = state.sensor.source_is_lsm6ds3trc ? 256.0f : 16.0f;
        fprintf(stderr, "[sensord] TEMP: %.1f C\n", 25.0f + (raw_t / tscale));
      }
    }

    usleep(9615);  // ~104Hz
  }

  state.sensor.close_dev();
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // 等待 I2C 总线就绪
  int bus_num = -1;
  for (int i = 0; i < 30; i++) {
    bus_num = find_ch347_i2c_bus();
    if (bus_num >= 0) {
      fprintf(stderr, "CH347 I2C bus detected after %d seconds: /dev/i2c-%d\n", i, bus_num);
      break;
    }
    if (i < 29) {
      fprintf(stderr, "CH347 I2C bus not found, waiting... (%d/30)\n", i + 1);
      sleep(1);
    }
  }

  if (bus_num < 0) {
    fprintf(stderr, "CH347 I2C bus not found after 30 seconds.\n");
    fprintf(stderr, "Please install AYSTARIK driver: sudo bash install_ch347_vcp.sh\n");
    return 1;
  }

  fprintf(stderr, "I2C bus ready, starting...\n");
  i2c_loop(bus_num);
  return 0;
}
