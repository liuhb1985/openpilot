// =============================================================================
// ubuntu347t: LSM6DS3 IMU via Linux native /dev/i2c-N with cereal publishing
//
// 特性：
//   - 启动时自动校准 IMU 装偏（2秒静态采样 + Rodrigues 旋转矩阵）
//   - 冻结检测：逐帧比较 raw bytes，CH347T/LSM6DS3 故障定位
//   - USB rebind 恢复：深层冻结时软复位 USB 芯片
//   - cereal 消息发布：accelerometer / gyroscope / temperatureSensor
// =============================================================================

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <dirent.h>
#include <chrono>
#include <climits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <array>

#include "cereal/messaging/messaging.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

ExitHandler do_exit;

// ============================================================
// 机器码授权验证模块
// 授权设备才能运行此程序
// 自动采集可用硬件信息，无需 root 权限
// ============================================================
namespace license {

// ========== 授权设备码白名单 ==========
static const std::vector<std::string> AUTHORIZED_DEVICE_CODES = {
    "551009E9F9532060",   // 当前开发机 (fallback: machine_id+hostname+mac)
    "A1B2C3D4E5F6G7H8",
    "C774B7F4A301037C",
    "5BC57BECDBA0BA57",
    "1957FD0C46BD2B3F",  //CRV
    "2D1920D2CE6E5D6A",  //20han
	    "DAE51FC3900592CF",  //Santafe
};

#if 0
static std::string read_file_trimmed(const char *path) {
    std::ifstream f(path);
    if (!f.is_open()) return "";
    std::string content;
    std::getline(f, content);
    while (!content.empty() && (content.back() == '\r' || content.back() == '\n' || content.back() == ' ')) {
        content.pop_back();
    }
    return content;
}

static std::string get_machine_id() {
    return read_file_trimmed("/etc/machine-id");
}

static std::string get_bios_uuid() {
    return read_file_trimmed("/sys/class/dmi/id/product_uuid");
}

static std::string get_hostname() {
    char buf[256] = {};
    gethostname(buf, sizeof(buf) - 1);
    return std::string(buf);
}

static std::string get_mac_address() {
    const char *net_path = "/sys/class/net";
    DIR *dir = opendir(net_path);
    if (dir) {
        std::vector<std::string> ifnames;
        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_name[0] == '.' || strcmp(entry->d_name, "lo") == 0) continue;
            ifnames.push_back(entry->d_name);
        }
        closedir(dir);

        std::sort(ifnames.begin(), ifnames.end());

        for (const auto &ifname : ifnames) {
            char addr_path[256];
            snprintf(addr_path, sizeof(addr_path), "%s/%s/address", net_path, ifname.c_str());
            std::string mac = read_file_trimmed(addr_path);
            if (!mac.empty() && mac != "00:00:00:00:00:00") {
                return mac;
            }
        }
    }

    std::ifstream f("/proc/net/arp");
    if (f.is_open()) {
        std::string line;
        std::vector<std::pair<std::string, std::string>> arp_entries;
        while (std::getline(f, line)) {
            if (line.find("IP address") != std::string::npos) continue;
            std::istringstream iss(line);
            std::string ip, hwtype, flags, mac, mask, device;
            if (iss >> ip >> hwtype >> flags >> mac >> mask >> device) {
                if (mac != "00:00:00:00:00:00" && device != "lo") {
                    arp_entries.push_back({device, mac});
                }
            }
        }
        std::sort(arp_entries.begin(), arp_entries.end());
        if (!arp_entries.empty()) {
            return arp_entries[0].second;
        }
    }
    return "";
}

static std::string compute_device_code(const std::vector<std::string> &factors) {
    std::string combined;
    for (const auto &f : factors) {
        if (!combined.empty()) combined += "|";
        combined += f.empty() ? "(none)" : f;
    }

    uint64_t hash = 14695981039346656037ULL;
    for (char c : combined) {
        hash ^= (uint64_t)(unsigned char)c;
        hash *= 1099511628211ULL;
    }

    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0') << std::setw(16) << hash;
    return oss.str();
}
#endif

static bool verify_license() {
    return true;
}

}  // namespace license

// CH347 USB VID/PID
static constexpr const char* CH347_USB_VID = "1a86";
static constexpr const char* CH347_USB_PID = "55db";
static constexpr uint8_t CH347_WHO_AM_I_REG = 0x0F;
static constexpr uint8_t CH347_LSM6_ADDRS[] = {0x6B, 0x6A};
static constexpr uint8_t CH347_WHO_AM_I_IDS[] = {0x69, 0x6A};

// 冻结检测阈值
static constexpr int FREEZE_EXIT_THRESHOLD = 8;
static constexpr int FREEZE_MAX_REBOUNDS = 2;
static constexpr int FROZEN_STALE_THRESHOLD = 4;

// IMU 加速度计启动自动校准参数
static constexpr float ACCEL_CALIB_DURATION = 2.0f;
static constexpr float ACCEL_CALIB_RATE = 104.0f;
static constexpr float ACCEL_CALIB_VARIANCE_MAX = 0.5f;
static constexpr float ACCEL_CALIB_GRAVITY = 9.81f;
static constexpr int ACCEL_CALIB_MIN_SAMPLES = 104;

// ---- 向量/矩阵辅助函数 ----

static void vec_normalize(const float v[3], float out[3]) {
  float n = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (n < 1e-12f) { out[0] = out[1] = out[2] = 0.0f; return; }
  out[0] = v[0] / n;
  out[1] = v[1] / n;
  out[2] = v[2] / n;
}

static float vec_dot(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void vec_cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

static void mat3_mul_vec3(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
  out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
  out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

static void mat3_identity(float R[9]) {
  memset(R, 0, 9 * sizeof(float));
  R[0] = R[4] = R[8] = 1.0f;
}

static void rotation_matrix_align(const float g_from[3], const float g_to[3], float R[9]) {
  float u[3], v[3];
  vec_normalize(g_from, u);
  vec_normalize(g_to, v);

  float w[3];
  vec_cross(u, v, w);
  float s = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
  float c = vec_dot(u, v);

  if (s < 1e-8f) {
    if (c > 0.0f) {
      mat3_identity(R);
    } else {
      mat3_identity(R);
      R[4] = -1.0f;
      R[8] = -1.0f;
    }
    return;
  }

  float K[9] = {
    0.0f,    -w[2],   w[1],
    w[2],    0.0f,   -w[0],
   -w[1],    w[0],    0.0f
  };

  float K2[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      K2[i*3 + j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        K2[i*3 + j] += K[i*3 + k] * K[k*3 + j];
      }
    }
  }

  float scale = (1.0f - c) / (s * s);
  for (int i = 0; i < 9; i++) {
    R[i] = (i % 4 == 0 ? 1.0f : 0.0f) + K[i] + K2[i] * scale;
  }
}

// ---- LinuxI2CLSM6 ----

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

    return ioctl(fd, I2C_RDWR, &ioctl_data) > 0;
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

    return ioctl(fd, I2C_RDWR, &ioctl_data) > 0;
  }

  bool read_u8(uint8_t reg, uint8_t& val) {
    return _i2c_rdwr(&reg, 1, &val, 1);
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
        uint8_t who = 0;
        if (!read_u8(CH347_WHO_AM_I_REG, who)) continue;
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
    if (!write_u8(0x12, 0x01)) return false;
    usleep(100000);
    if (!write_u8(0x12, 0x44)) return false;
    if (!write_u8(0x10, 0x40)) return false;
    if (!write_u8(0x11, 0x40)) return false;
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

  void shutdown() { close_dev(); }

  cereal::SensorEventData::SensorSource sensor_source() const {
    return source_is_lsm6ds3trc
      ? cereal::SensorEventData::SensorSource::LSM6DS3TRC
      : cereal::SensorEventData::SensorSource::LSM6DS3;
  }
};

// ---- I2C 总线查找 ----

static int find_ch347_i2c_bus() {
  DIR* dir = opendir("/sys/bus/i2c/devices");
  if (dir) {
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strncmp(entry->d_name, "i2c-", 4) != 0) continue;
      char name_path[256];
      snprintf(name_path, sizeof(name_path), "/sys/bus/i2c/devices/%s/name", entry->d_name);
      std::ifstream name_file(name_path);
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

  for (int bus = 0; bus < 10; bus++) {
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", bus);
    int fd = ::open(dev_path, O_RDWR);
    if (fd < 0) continue;

    for (size_t i = 0; i < sizeof(CH347_LSM6_ADDRS); i++) {
      uint8_t addr = CH347_LSM6_ADDRS[i];
      if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0) continue;

      uint8_t wbuf = CH347_WHO_AM_I_REG;
      uint8_t rbuf = 0;
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

      if (ioctl(fd, I2C_RDWR, &ioctl_data) > 0) {
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

// ---- USB rebind ----

static std::string find_ch347_usb_device() {
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

static bool usb_rebind_ch347(const std::string& usb_dev) {
  char unbind_path[256], bind_path[256];
  snprintf(unbind_path, sizeof(unbind_path), "/sys/bus/usb/drivers/usb/unbind");
  snprintf(bind_path, sizeof(bind_path), "/sys/bus/usb/drivers/usb/bind");

  {
    std::ofstream f(unbind_path);
    if (f.is_open()) f << usb_dev;
  }
  usleep(100000);

  {
    std::ofstream f(bind_path);
    if (f.is_open()) f << usb_dev;
  }
  usleep(1000000);

  return find_ch347_i2c_bus() >= 0;
}

// ---- IMU 装偏校准 ----

static void calibrate_imu_mounting(LinuxI2CLSM6& sensor, float R[9]) {
  LOGW("IMU mounting calib: collecting static samples...");

  std::vector<std::array<float, 3>> samples;
  auto t_start = std::chrono::steady_clock::now();

  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
        std::chrono::steady_clock::now() - t_start).count();
    if (elapsed >= ACCEL_CALIB_DURATION) break;

    uint8_t status = 0;
    if (sensor.read_u8(0x1E, status) && (status & 0x01)) {
      uint8_t b[6];
      if (sensor.read_block(0x28, b, 6)) {
        float x = (int16_t)((uint16_t)b[1] << 8 | b[0]);
        float y = (int16_t)((uint16_t)b[3] << 8 | b[2]);
        float z = (int16_t)((uint16_t)b[5] << 8 | b[4]);
        float scale = 9.81f * 2.0f / (1 << 15);
        std::array<float, 3> s;
        s[0] = y * scale;
        s[1] = -x * scale;
        s[2] = z * scale;
        samples.push_back(s);
      }
    }
    usleep(static_cast<int>(1.0f / ACCEL_CALIB_RATE * 1000000.0f));
  }

  if ((int)samples.size() < ACCEL_CALIB_MIN_SAMPLES) {
    LOGW("IMU mounting calib: not enough samples (%zu), using identity", samples.size());
    mat3_identity(R);
    return;
  }

  float g_meas[3] = {0, 0, 0};
  for (const auto& s : samples) {
    g_meas[0] += s[0];
    g_meas[1] += s[1];
    g_meas[2] += s[2];
  }
  size_t n = samples.size();
  g_meas[0] /= n;
  g_meas[1] /= n;
  g_meas[2] /= n;

  float var[3] = {0, 0, 0};
  for (const auto& s : samples) {
    float d;
    d = s[0] - g_meas[0]; var[0] += d * d;
    d = s[1] - g_meas[1]; var[1] += d * d;
    d = s[2] - g_meas[2]; var[2] += d * d;
  }
  var[0] /= n; var[1] /= n; var[2] /= n;

  float max_var = var[0];
  if (var[1] > max_var) max_var = var[1];
  if (var[2] > max_var) max_var = var[2];

  if (max_var > ACCEL_CALIB_VARIANCE_MAX) {
    LOGW("IMU mounting calib: vehicle moving (var=[%.4f,%.4f,%.4f]), using identity",
         var[0], var[1], var[2]);
    mat3_identity(R);
    return;
  }

  float target[3] = {ACCEL_CALIB_GRAVITY, 0.0f, 0.0f};
  rotation_matrix_align(g_meas, target, R);

  LOGW("IMU mounting calib: done. g_meas=[%.4f,%.4f,%.4f] var=[%.4f,%.4f,%.4f]",
       g_meas[0], g_meas[1], g_meas[2], var[0], var[1], var[2]);
}

// ---- cereal 消息发布 ----

static void publish_accelerometer(PubMaster& pm,
                                   cereal::SensorEventData::SensorSource source,
                                   const float v[3]) {
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto se = evt.initAccelerometer();
  se.setVersion(1);
  se.setSensor(1);
  se.setType(1);
  se.setSource(source);
  se.setTimestamp(evt.getLogMonoTime());
  auto acc = se.initAcceleration();
  auto vv = acc.initV(3);
  vv.set(0, v[0]);
  vv.set(1, v[1]);
  vv.set(2, v[2]);
  acc.setStatus(1);
  pm.send("accelerometer", msg);
}

static void publish_gyroscope(PubMaster& pm,
                               cereal::SensorEventData::SensorSource source,
                               const float v[3]) {
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto se = evt.initGyroscope();
  se.setVersion(2);
  se.setSensor(5);
  se.setType(16);
  se.setSource(source);
  se.setTimestamp(evt.getLogMonoTime());
  auto gyro = se.initGyroUncalibrated();
  auto vv = gyro.initV(3);
  vv.set(0, v[0]);
  vv.set(1, v[1]);
  vv.set(2, v[2]);
  gyro.setStatus(1);
  pm.send("gyroscope", msg);
}

static void publish_temperature(PubMaster& pm,
                                 cereal::SensorEventData::SensorSource source,
                                 float raw_t) {
  float tscale = (source == cereal::SensorEventData::SensorSource::LSM6DS3) ? 16.0f : 256.0f;
  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto se = evt.initTemperatureSensor();
  se.setVersion(1);
  se.setSource(source);
  se.setTimestamp(evt.getLogMonoTime());
  se.setTemperature(25.0f + raw_t / tscale);
  pm.send("temperatureSensor", msg);
}

// ---- I2C 主循环状态 ----

struct I2CState {
  LinuxI2CLSM6 sensor;
  float imu_calib_R[9] = {1,0,0, 0,1,0, 0,0,1};
  uint8_t last_acc_raw[6] = {0};
  uint8_t last_gyro_raw[6] = {0};
  bool has_acc_raw = false;
  bool has_gyro_raw = false;
  float last_acc_v[3] = {0};
  float last_gyro_v[3] = {0};
  int consecutive_errors = 0;
  int freeze_counter = 0;
  int rebind_count = 0;
  int freeze_freq_counter = 0;
  uint64_t freeze_freq_window_ns = 0;
};

static bool sensor_open_retry(LinuxI2CLSM6& sensor, int bus_num, int max_retries = 10) {
  sensor.bus_num = bus_num;
  for (int attempt = 0; attempt < max_retries; attempt++) {
    if (sensor.open_dev()) return true;
    if (attempt < max_retries - 1) {
      LOGW("I2C: sensor.open() failed (%d/%d), retrying in 2s...", attempt + 1, max_retries);
      util::sleep_for(2000);
    }
  }
  return false;
}

// ---- 主循环 ----

static void i2c_loop(int bus_num, PubMaster& pm) {
  I2CState state;
  RateKeeper rk("ubuntu347td", 104.0f);

  if (!sensor_open_retry(state.sensor, bus_num, 10)) {
    LOGE("I2C: sensor.open() failed after 10 retries");
    return;
  }
  if (!state.sensor.init_sensor()) {
    LOGE("I2C: failed to initialize sensor");
    state.sensor.shutdown();
    return;
  }

  calibrate_imu_mounting(state.sensor, state.imu_calib_R);

  int temp_ticks = 0;
  uint64_t last_log_t = 0;
  uint64_t stale_log_t = 0;
  uint64_t freeze_log_t = 0;

  auto source = state.sensor.sensor_source();

  while (!do_exit) {
    bool i2c_error = false;
    bool fresh_acc_valid = false;
    bool fresh_gyro_valid = false;
    float fresh_acc[3] = {0};
    float fresh_gyro[3] = {0};
    uint8_t fresh_acc_raw[6] = {0};
    uint8_t fresh_gyro_raw[6] = {0};

    // 1. STATUS_REG (0x1E)
    uint8_t status = 0;
    if (!state.sensor.read_u8(0x1E, status)) {
      i2c_error = true;
    }

    // 2. 加速度计
    if (status & 0x01) {
      if (state.sensor.read_block(0x28, fresh_acc_raw, 6)) {
        float x = (int16_t)((uint16_t)fresh_acc_raw[1] << 8 | fresh_acc_raw[0]);
        float y = (int16_t)((uint16_t)fresh_acc_raw[3] << 8 | fresh_acc_raw[2]);
        float z = (int16_t)((uint16_t)fresh_acc_raw[5] << 8 | fresh_acc_raw[4]);
        float scale = 9.81f * 2.0f / (1 << 15);
        float raw[3] = {-y * scale, x * scale, z * scale};
        mat3_mul_vec3(state.imu_calib_R, raw, fresh_acc);
        fresh_acc_valid = true;
        memcpy(state.last_acc_v, fresh_acc, sizeof(fresh_acc));
      } else {
        i2c_error = true;
      }
    }

    // 3. 陀螺仪
    if (status & 0x02) {
      if (state.sensor.read_block(0x22, fresh_gyro_raw, 6)) {
        float x = (int16_t)((uint16_t)fresh_gyro_raw[1] << 8 | fresh_gyro_raw[0]);
        float y = (int16_t)((uint16_t)fresh_gyro_raw[3] << 8 | fresh_gyro_raw[2]);
        float z = (int16_t)((uint16_t)fresh_gyro_raw[5] << 8 | fresh_gyro_raw[4]);
        float scale = (8.75f / 1000.0f) * (M_PI / 180.0f);
        float raw[3] = {-y * scale, x * scale, z * scale};
        mat3_mul_vec3(state.imu_calib_R, raw, fresh_gyro);
        fresh_gyro_valid = true;
        memcpy(state.last_gyro_v, fresh_gyro, sizeof(fresh_gyro));
      } else {
        i2c_error = true;
      }
    }

    // 4. 错误计数
    if (i2c_error) {
      state.consecutive_errors++;
    } else {
      state.consecutive_errors = 0;
    }

    // 5. 冻结检测
    bool acc_frozen = (fresh_acc_valid && state.has_acc_raw &&
                       memcmp(fresh_acc_raw, state.last_acc_raw, 6) == 0);
    bool gyro_frozen = (fresh_gyro_valid && state.has_gyro_raw &&
                        memcmp(fresh_gyro_raw, state.last_gyro_raw, 6) == 0);
    bool any_frozen = acc_frozen || gyro_frozen;
    bool acc_unfrozen = (fresh_acc_valid && state.has_acc_raw &&
                         memcmp(fresh_acc_raw, state.last_acc_raw, 6) != 0);
    bool gyro_unfrozen = (fresh_gyro_valid && state.has_gyro_raw &&
                          memcmp(fresh_gyro_raw, state.last_gyro_raw, 6) != 0);
    bool any_unfrozen = acc_unfrozen || gyro_unfrozen;

    if (any_frozen) {
      state.freeze_counter++;
      uint64_t now = nanos_since_boot();

      if (state.freeze_counter >= 3) {
        if (state.freeze_counter == 3 || state.freeze_counter == 7) {
          uint8_t who = 0;
          bool who_ok = false;
          if (state.sensor.read_u8(0x0F, who)) {
            for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
              if (who == CH347_WHO_AM_I_IDS[j]) { who_ok = true; break; }
            }
          }

          if (i2c_error || !who_ok) {
            LOGE("I2C: FREEZE fault=CH347T freeze=%d WHO=0x%02X", state.freeze_counter, who);
          } else {
            uint8_t ctrl3 = 0, ctrl1 = 0, ctrl2 = 0;
            state.sensor.read_u8(0x12, ctrl3);
            state.sensor.read_u8(0x10, ctrl1);
            state.sensor.read_u8(0x11, ctrl2);
            LOGE("I2C: FREEZE fault=LSM6DS3 freeze=%d WHO=0x%02X "
                 "CTRL3=0x%02X CTRL1=0x%02X CTRL2=0x%02X",
                 state.freeze_counter, who, ctrl3, ctrl1, ctrl2);
          }
        }

        if (now - freeze_log_t > 1e9) {
          LOGW("I2C: Data frozen! freeze_counter=%d acc_frozen=%d gyro_frozen=%d i2c_error=%d",
               state.freeze_counter, acc_frozen, gyro_frozen, i2c_error);
          freeze_log_t = now;
        }
      }
    } else if (any_unfrozen) {
      if (state.freeze_counter > 0) {
        uint64_t now = nanos_since_boot();
        if (now - state.freeze_freq_window_ns > 60ULL * (uint64_t)1e9) {
          state.freeze_freq_counter = 0;
          state.freeze_freq_window_ns = now;
        }
        state.freeze_freq_counter++;
        if (state.freeze_freq_counter >= 5) {
          LOGW("I2C: CH347T intermittent freeze detected (%d times in 60s), check hardware",
               state.freeze_freq_counter);
          state.freeze_freq_counter = 0;
        }
        LOGW("I2C: Data unfrozen after %d frames", state.freeze_counter);
      }
      state.freeze_counter = 0;
    }

    // 6. 冻结恢复（USB rebind）
    if (state.freeze_counter >= FREEZE_EXIT_THRESHOLD) {
      state.rebind_count++;
      if (state.rebind_count <= FREEZE_MAX_REBOUNDS) {
        LOGE("I2C: Data frozen for %d frames, attempting USB rebind (%d/%d)",
             state.freeze_counter, state.rebind_count, FREEZE_MAX_REBOUNDS);

        std::string usb_dev = find_ch347_usb_device();
        if (!usb_dev.empty() && usb_rebind_ch347(usb_dev)) {
          int new_bus = find_ch347_i2c_bus();
          if (new_bus >= 0) {
            state.sensor.shutdown();
            state.sensor.bus_num = new_bus;
            if (state.sensor.open_dev() && state.sensor.init_sensor()) {
              usleep(200000);

              uint8_t test1[6], test2[6];
              if (!state.sensor.read_block(0x22, test1, 6)) {
                LOGE("I2C: LSM6DS3 read test failed after rebind, exiting");
                state.sensor.shutdown();
                break;
              }
              usleep(100000);
              if (!state.sensor.read_block(0x22, test2, 6)) {
                LOGE("I2C: LSM6DS3 read test failed after rebind, exiting");
                state.sensor.shutdown();
                break;
              }
              if (memcmp(test1, test2, 6) == 0) {
                LOGE("I2C: LSM6DS3 still frozen after rebind (data identical), exiting");
                state.sensor.shutdown();
                break;
              }

              state.freeze_counter = 0;
              state.consecutive_errors = 0;
              state.rebind_count = 0;
              state.has_acc_raw = false;
              state.has_gyro_raw = false;
              memset(state.last_acc_v, 0, sizeof(state.last_acc_v));
              memset(state.last_gyro_v, 0, sizeof(state.last_gyro_v));
              source = state.sensor.sensor_source();
              LOGW("I2C: Sensor recovered after USB rebind, new bus=/dev/i2c-%d", new_bus);
              continue;
            }
          }
        }
        LOGW("I2C: USB rebind failed (usb_dev=%s)",
             usb_dev.empty() ? "none" : usb_dev.c_str());
      }
      LOGE("I2C: Rebind exhausted (%d/%d), exiting",
           state.rebind_count, FREEZE_MAX_REBOUNDS);
      break;
    }

    // 保存原始 bytes 供下一帧比较
    if (fresh_acc_valid) {
      memcpy(state.last_acc_raw, fresh_acc_raw, 6);
      state.has_acc_raw = true;
    }
    if (fresh_gyro_valid) {
      memcpy(state.last_gyro_raw, fresh_gyro_raw, 6);
      state.has_gyro_raw = true;
    }

    // 7. cereal 消息发布
    bool serious_frozen = any_frozen && state.freeze_counter >= FROZEN_STALE_THRESHOLD;
    bool should_send_stale = i2c_error || (any_frozen && !serious_frozen);

    const float* acc_to_send = fresh_acc_valid ? fresh_acc :
                                (should_send_stale ? state.last_acc_v : nullptr);
    if (acc_to_send) {
      publish_accelerometer(pm, source, acc_to_send);
    }

    const float* gyro_to_send = fresh_gyro_valid ? fresh_gyro :
                                 (should_send_stale ? state.last_gyro_v : nullptr);
    if (gyro_to_send) {
      publish_gyroscope(pm, source, gyro_to_send);
    }

    if (i2c_error) {
      uint64_t now = nanos_since_boot();
      if (now - stale_log_t > 1e9) {
        LOGW("I2C: I2C error, sending stale data (consecutive=%d)", state.consecutive_errors);
        stale_log_t = now;
      }
    } else if (any_frozen && !serious_frozen) {
      uint64_t now = nanos_since_boot();
      if (now - stale_log_t > 1e9) {
        LOGW("I2C: Data frozen briefly, sending stale (freeze_counter=%d)", state.freeze_counter);
        stale_log_t = now;
      }
    } else if (serious_frozen) {
      uint64_t now = nanos_since_boot();
      if (now - stale_log_t > 1e9) {
        LOGW("I2C: Data frozen seriously, NOT sending (freeze_counter=%d)", state.freeze_counter);
        stale_log_t = now;
      }
    }

    // 8. 控制台调试日志
    {
      uint64_t now = nanos_since_boot();
      if (now - last_log_t > 1e9) {
        if (serious_frozen) {
          uint8_t who = 0;
          bool who_ok = false;
          if (state.sensor.read_u8(0x0F, who)) {
            for (size_t j = 0; j < sizeof(CH347_WHO_AM_I_IDS); j++) {
              if (who == CH347_WHO_AM_I_IDS[j]) { who_ok = true; break; }
            }
          }
          const char* fault = (i2c_error || !who_ok) ? "CH347T" : "LSM6DS3";
          fprintf(stderr, "[ubuntu347t] FROZEN! fault=%s freeze=%d, data halted\n",
                  fault, state.freeze_counter);
        } else if (state.has_acc_raw) {
          const char* freeze_tag = (state.freeze_counter > 0) ? " (freeze)" : "";
          fprintf(stderr, "[ubuntu347t] ACC: [%9.4f, %9.4f, %9.4f] m/s2%s\n",
                  state.last_acc_v[0], state.last_acc_v[1], state.last_acc_v[2], freeze_tag);
          fprintf(stderr, "[ubuntu347t] GYRO: [%9.5f, %9.5f, %9.5f] rad/s\n",
                  state.last_gyro_v[0], state.last_gyro_v[1], state.last_gyro_v[2]);
        }
        fflush(stderr);
        last_log_t = now;
      }
    }

    // 9. 温度读取（每10帧）
    temp_ticks++;
    if (temp_ticks >= 10) {
      temp_ticks = 0;
      uint8_t tb[2];
      if (state.sensor.read_block(0x20, tb, 2)) {
        int16_t raw_t = (int16_t)((uint16_t)tb[1] << 8 | tb[0]);
        publish_temperature(pm, source, raw_t);
      }
    }

    // 10. 维持 104Hz
    rk.keepTime();
  }

  state.sensor.shutdown();
}

int main() {
  // 机器码授权验证（未授权设备无法运行）
  if (!license::verify_license()) {
    std::srand((unsigned int)std::time(nullptr));
    while (!do_exit) {
      int timeout = 500 + std::rand() % 500;
      fprintf(stderr, "CH347 not responding, retry in %dms...\n", timeout);
      for (int i = 0; i < 10 && !do_exit; i++) {
        usleep(50000);
      }
    }
    return 0;
  }

  setpriority(PRIO_PROCESS, 0, -15);
  PubMaster publisher({"accelerometer", "gyroscope", "temperatureSensor"});

  int bus_num = -1;
  for (int i = 0; i < 30 && !do_exit; i++) {
    bus_num = find_ch347_i2c_bus();
    if (bus_num >= 0) {
      LOGW("CH347 I2C bus detected after %d seconds: /dev/i2c-%d", i, bus_num);
      break;
    }
    if (i < 29) {
      LOGW("CH347 I2C bus not found, waiting... (%d/30)", i + 1);
      util::sleep_for(1000);
    }
  }

  if (bus_num < 0) {
    LOGE("CH347 I2C bus not found after 30 seconds.");
    LOGE("Please install AYSTARIK driver: sudo bash install_ch347_vcp.sh");
    LOGE("Or check: ls /dev/i2c-* && lsmod | grep ch347");
    return 1;
  }

  LOGW("I2C bus ready, waiting 1s...");
  util::sleep_for(1000);
  i2c_loop(bus_num, publisher);
  LOGW("ubuntu347t exiting");
  return 0;
}
