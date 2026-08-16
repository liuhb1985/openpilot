// =============================================================================
// ch347t_reconstructed.cc
//
// RECONSTRUCTION from:
//   1) /mnt/data/ch347t       (ELF64, DWARF, unstripped)
//   2) /mnt/data/ubuntu347t.cc (user-provided companion source)
//
// IMPORTANT:
//   This is a SOURCE-LEVEL RECONSTRUCTION, not a byte-for-byte recovery of
//   the original source. The ELF preserves DWARF symbols, types, source
//   paths, declaration line numbers, constants and strings, so the CH347/I2C
//   implementation can be reconstructed with high confidence. Comments and
//   some control-flow details are inferred.
// =============================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <glob.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "cereal/messaging/messaging.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#ifndef INFO
#define INFO CLOUDLOG_INFO
#endif
#ifndef WARNING
#define WARNING CLOUDLOG_WARNING
#endif
#ifndef ERROR
#define ERROR CLOUDLOG_ERROR
#endif
#ifndef DEBUG
#define DEBUG CLOUDLOG_DEBUG
#endif

ExitHandler do_exit;

// -----------------------------------------------------------------------------
// Constants recovered from the ELF
// -----------------------------------------------------------------------------

static constexpr uint8_t WHO_AM_I_REG = 0x0F;
static constexpr uint8_t LSM6_ADDR_A = 0x6B;
static constexpr uint8_t LSM6_ADDR_B = 0x6A;
static constexpr uint8_t LSM6_WHO_AM_I = 0x69;
static constexpr uint8_t LSM6TRC_WHO_AM_I = 0x6A;

static constexpr float ACCEL_SCALE = 0.0005987549084238708f;
static constexpr float GYRO_SCALE  = 0.00015271631127689034f;
static constexpr float TEMP_SCALE  = 1.0f / 16.0f;
static constexpr float TEMP_OFFSET = 25.0f;

static constexpr int DEFAULT_I2C_BUS = 19;

// The ELF contains these exact strings/thresholds.
static constexpr int REINIT_ERROR_THRESHOLD = 4;
static constexpr int MAX_REINIT_ATTEMPTS = 2;

// -----------------------------------------------------------------------------
// Abstract LSM6DS3 interface
//
// DWARF confirms this class exists in the original translation unit at about
// line 231, with these pure virtual methods.
// -----------------------------------------------------------------------------

class LSM6DS3 {
public:
  virtual ~LSM6DS3() = default;

  virtual bool open() = 0;
  virtual bool init_sensor() = 0;
  virtual bool shutdown() = 0;
  virtual uint8_t read_u8(uint8_t reg) = 0;
  virtual void write_u8(uint8_t reg, uint8_t value) = 0;

  // Original ABI returns a vector-like object for block reads.
  virtual std::vector<uint8_t> read_block(uint8_t reg, int len) = 0;

  virtual cereal::SensorEventData::SensorSource source() const = 0;
};

// -----------------------------------------------------------------------------
// CH347 dynamic API typedefs
//
// These signatures are recovered from DWARF function types and call sites.
// -----------------------------------------------------------------------------

using CH347OpenDevice_t =
    int (*)(const char *dev_path);

using CH347CloseDevice_t =
    bool (*)(int fd);

using CH34xSetTimeout_t =
    bool (*)(int fd, int read_timeout_ms, int write_timeout_ms);

using CH347I2C_Set_t =
    bool (*)(int fd, int kbps);

using CH347I2C_SetIgnoreNack_t =
    bool (*)(int fd, uint8_t enable);

using CH347I2C_SetStretch_t =
    bool (*)(int fd, bool enable);

using CH347StreamI2C_t =
    bool (*)(int fd, int write_len, const uint8_t *write_buf,
             int read_len, uint8_t *read_buf);

using CH347StreamI2C_RetAck_t =
    bool (*)(int fd, int write_len, const uint8_t *write_buf,
             int read_len, uint8_t *read_buf, int *ret_len);

// -----------------------------------------------------------------------------
// CH347LSM6
//
// DWARF layout recovered exactly:
//
//   sizeof(CH347LSM6) = 120
//   dev_path_              @ +8
//   lib_handle_            @ +40
//   fd_                    @ +48
//   addr_                  @ +52
//   source_                @ +54
//   open_dev_              @ +56
//   close_dev_             @ +64
//   set_timeout_           @ +72
//   i2c_set_               @ +80
//   i2c_set_ignore_nack_   @ +88
//   i2c_set_stretch_       @ +96
//   stream_i2c_            @ +104
//   stream_i2c_ret_ack_    @ +112
// -----------------------------------------------------------------------------

class CH347LSM6 final : public LSM6DS3 {
public:
  CH347LSM6(const std::string &dev_path, void *lib_handle)
      : dev_path_(dev_path),
        lib_handle_(lib_handle),
        fd_(-1),
        addr_(0),
        source_(cereal::SensorEventData::SensorSource::LSM6DS3) {

    open_dev_ = reinterpret_cast<CH347OpenDevice_t>(
        dlsym(lib_handle_, "CH347OpenDevice"));
    close_dev_ = reinterpret_cast<CH347CloseDevice_t>(
        dlsym(lib_handle_, "CH347CloseDevice"));
    set_timeout_ = reinterpret_cast<CH34xSetTimeout_t>(
        dlsym(lib_handle_, "CH34xSetTimeout"));
    i2c_set_ = reinterpret_cast<CH347I2C_Set_t>(
        dlsym(lib_handle_, "CH347I2C_Set"));
    i2c_set_ignore_nack_ = reinterpret_cast<CH347I2C_SetIgnoreNack_t>(
        dlsym(lib_handle_, "CH347I2C_SetIgnoreNack"));
    i2c_set_stretch_ = reinterpret_cast<CH347I2C_SetStretch_t>(
        dlsym(lib_handle_, "CH347I2C_SetStretch"));
    stream_i2c_ = reinterpret_cast<CH347StreamI2C_t>(
        dlsym(lib_handle_, "CH347StreamI2C"));
    stream_i2c_ret_ack_ = reinterpret_cast<CH347StreamI2C_RetAck_t>(
        dlsym(lib_handle_, "CH347StreamI2C_RetAck"));
  }

  ~CH347LSM6() override = default;

  bool open() override;
  bool init_sensor() override;
  bool shutdown() override;

  uint8_t read_u8(uint8_t reg) override;
  void write_u8(uint8_t reg, uint8_t value) override;

  std::vector<uint8_t> read_block(uint8_t reg, int len) override;

  cereal::SensorEventData::SensorSource source() const override {
    return source_;
  }

  // The original binary has this helper as a separate non-virtual method.
  std::vector<uint8_t> stream_read(const uint8_t *write_buf,
                                   int write_len,
                                   int read_len);

private:
  std::string dev_path_;
  void *lib_handle_;
  int fd_;
  uint8_t addr_;
  cereal::SensorEventData::SensorSource source_;

  CH347OpenDevice_t open_dev_;
  CH347CloseDevice_t close_dev_;
  CH34xSetTimeout_t set_timeout_;
  CH347I2C_Set_t i2c_set_;
  CH347I2C_SetIgnoreNack_t i2c_set_ignore_nack_;
  CH347I2C_SetStretch_t i2c_set_stretch_;
  CH347StreamI2C_t stream_i2c_;
  CH347StreamI2C_RetAck_t stream_i2c_ret_ack_;
};

// -----------------------------------------------------------------------------
// CH347 open()
//
// Recovered behavior:
//   CH347OpenDevice(dev_path)
//   CH34xSetTimeout(fd, 2000, 2000)
//   CH347I2C_Set(fd, 1)
//   CH347I2C_SetIgnoreNack(fd, 1)
//   CH347I2C_SetStretch(fd, 1)
//   probe 0x6B then 0x6A, retry 8 times, sleep 20 ms
// -----------------------------------------------------------------------------

bool CH347LSM6::open() {
  fd_ = open_dev_(dev_path_.c_str());
  if (fd_ < 0) {
    throw std::runtime_error("failed to open " + dev_path_);
  }

  if (!set_timeout_(fd_, 2000, 2000)) {
    throw std::runtime_error("CH34xSetTimeout failed");
  }

  if (!i2c_set_(fd_, 1)) {
    throw std::runtime_error("CH347I2C_Set failed");
  }

  (void)i2c_set_ignore_nack_(fd_, 1);
  (void)i2c_set_stretch_(fd_, true);

  struct timespec ts{0, 0x01312d00}; // 20 ms

  for (int attempt = 0; attempt < 8; ++attempt) {
    for (uint8_t addr : {LSM6_ADDR_A, LSM6_ADDR_B}) {
      uint8_t reg = WHO_AM_I_REG;
      auto b = stream_read(&reg, 1, 1);
      if (b.empty()) continue;

      uint8_t who = b[0];
      if (who == LSM6_WHO_AM_I || who == LSM6TRC_WHO_AM_I) {
        addr_ = addr;

        if (who == LSM6TRC_WHO_AM_I) {
          source_ = cereal::SensorEventData::SensorSource::LSM6DS3TRC;
        } else {
          source_ = cereal::SensorEventData::SensorSource::LSM6DS3;
        }

        cloudlog_e(
            INFO,
            "system/sensord/ch347t.cc",
            0x11f,
            "open",
            "CH347 LSM6DS3 detected at addr=0x%02X, who=0x%02X",
            addr_, who);
        return true;
      }
    }

    while (nanosleep(&ts, &ts) < 0 && errno == EINTR) {}
  }

  throw std::runtime_error("CH347 LSM6DS3 not detected on 0x6A/0x6B");
}

// -----------------------------------------------------------------------------
// init_sensor()
//
// The binary writes:
//   0x12 <- 0x01
//   sleep 100 ms
//   0x12 <- 0x04
//   0x10 <- 0x40
//   0x11 <- 0x40
// -----------------------------------------------------------------------------

bool CH347LSM6::init_sensor() {
  write_u8(0x12, 0x01);

  struct timespec ts{0, 0x05f5e100}; // 100 ms
  while (nanosleep(&ts, &ts) < 0 && errno == EINTR) {}

  write_u8(0x12, 0x04);
  write_u8(0x10, 0x40);
  write_u8(0x11, 0x40);
  return true;
}

bool CH347LSM6::shutdown() {
  if (fd_ >= 0) {
    try { write_u8(0x10, 0); } catch (...) {}
    try { write_u8(0x11, 0); } catch (...) {}

    try {
      close_dev_(fd_);
    } catch (...) {}

    fd_ = -1;
  }
  return true;
}

uint8_t CH347LSM6::read_u8(uint8_t reg) {
  auto b = stream_read(&reg, 1, 1);
  if (b.empty()) {
    throw std::runtime_error("CH347 I2C read failed");
  }
  return b[0];
}

void CH347LSM6::write_u8(uint8_t reg, uint8_t value) {
  uint8_t buf[3] = {
    static_cast<uint8_t>(addr_ << 1),
    reg,
    value
  };

  if (!stream_i2c_(fd_, 3, buf, 0, nullptr)) {
    throw std::runtime_error("CH347 I2C write failed");
  }
}

std::vector<uint8_t> CH347LSM6::read_block(uint8_t reg, int len) {
  uint8_t addr_w = static_cast<uint8_t>(addr_ << 1);

  int ret_len = 0;
  std::vector<uint8_t> out;
  if (len < 0) throw std::length_error("CH347 I2C read length");

  out.resize(static_cast<size_t>(len));

  if (!stream_i2c_ret_ack_(fd_, 2, &addr_w, 0, nullptr, &ret_len)) {
    throw std::runtime_error("CH347 I2C block write address failed");
  }

  uint8_t reg_buf[1] = {reg};
  if (!stream_i2c_ret_ack_(fd_, 1, reg_buf, len, out.data(), &ret_len)) {
    throw std::runtime_error("CH347 I2C block read data failed");
  }

  return out;
}

std::vector<uint8_t> CH347LSM6::stream_read(const uint8_t *write_buf,
                                            int write_len,
                                            int read_len) {
  if (read_len < 0) throw std::length_error("CH347 I2C read length");

  std::vector<uint8_t> out(static_cast<size_t>(read_len));
  int ret_len = 0;

  if (!stream_i2c_ret_ack_(fd_, write_len, write_buf,
                           read_len, out.data(), &ret_len)) {
    throw std::runtime_error("CH347 I2C read failed");
  }

  return out;
}

// -----------------------------------------------------------------------------
// Direct Linux I2C backend
//
// DWARF:
//   class I2CLSM6 starts around line 393
//   members bus_ @ 475, addr_ @ 476, fd_ @ 477, source_ @ 478
// -----------------------------------------------------------------------------

class I2CLSM6 final : public LSM6DS3 {
public:
  I2CLSM6(int bus, uint8_t addr)
      : bus_(bus), addr_(addr), fd_(-1),
        source_(cereal::SensorEventData::SensorSource::LSM6DS3) {}

  ~I2CLSM6() override = default;

  bool open() override;
  bool init_sensor() override;
  bool shutdown() override;

  uint8_t read_u8(uint8_t reg) override;
  void write_u8(uint8_t reg, uint8_t value) override;
  std::vector<uint8_t> read_block(uint8_t reg, int len) override;

  cereal::SensorEventData::SensorSource source() const override {
    return source_;
  }

private:
  int bus_;
  uint8_t addr_;
  int fd_;
  cereal::SensorEventData::SensorSource source_;
};

bool I2CLSM6::open() {
  char dev_path[32];
  snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", bus_);

  fd_ = ::open(dev_path, O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error(
        std::string("failed to open ") + dev_path);
  }

  if (ioctl(fd_, I2C_SLAVE_FORCE, addr_) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("I2C_SLAVE failed");
  }

  return true;
}

bool I2CLSM6::init_sensor() {
  write_u8(0x12, 0x01);

  struct timespec ts{0, 0x05f5e100};
  while (nanosleep(&ts, &ts) < 0 && errno == EINTR) {}

  write_u8(0x12, 0x04);
  write_u8(0x10, 0x40);
  write_u8(0x11, 0x40);
  return true;
}

bool I2CLSM6::shutdown() {
  if (fd_ >= 0) {
    try { write_u8(0x10, 0); } catch (...) {}
    try { write_u8(0x11, 0); } catch (...) {}
    ::close(fd_);
    fd_ = -1;
  }
  return true;
}

uint8_t I2CLSM6::read_u8(uint8_t reg) {
  if (::write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("I2C write reg failed");
  }

  uint8_t val = 0;
  if (::read(fd_, &val, 1) != 1) {
    throw std::runtime_error("I2C_RDWR failed");
  }
  return val;
}

void I2CLSM6::write_u8(uint8_t reg, uint8_t value) {
  uint8_t b[2] = {reg, value};
  if (::write(fd_, b, 2) != 2) {
    throw std::runtime_error("I2C write reg failed");
  }
}

std::vector<uint8_t> I2CLSM6::read_block(uint8_t reg, int len) {
  if (len < 0) throw std::length_error("I2C read length");

  if (::write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("I2C write reg failed");
  }

  std::vector<uint8_t> out(static_cast<size_t>(len));
  if (::read(fd_, out.data(), len) != len) {
    throw std::runtime_error("I2C_RDWR failed");
  }

  return out;
}

// -----------------------------------------------------------------------------
// CH347 library path
//
// The ELF contains:
//   /proc/self/exe
//   x86_64
//   amd64
//   arm-gnueabihf
//   arm-gnueabi
//   aarch64
//   /third_party/ch347/lib/
//   /third_party/ch347/lib/x64/dynamic/libch347.so
// -----------------------------------------------------------------------------

static std::string get_ch347_lib_path() {
  char exe_buf[4096] = {};
  ssize_t n = ::readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
  if (n <= 0) return "";

  exe_buf[n] = '\0';

  std::filesystem::path exe_path(exe_buf);
  std::filesystem::path project_root = exe_path.parent_path();

  // sensord/ch347t -> project root
  for (int i = 0; i < 3 && !project_root.empty(); ++i) {
    project_root = project_root.parent_path();
  }

#if defined(__x86_64__)
  const char *arch_dir = "x64";
#elif defined(__aarch64__)
  const char *arch_dir = "aarch64";
#elif defined(__arm__)
  const char *arch_dir = "arm";
#else
  const char *arch_dir = "x64";
#endif

  std::filesystem::path p =
      project_root / "third_party" / "ch347" / "lib" /
      arch_dir / "dynamic" / "libch347.so";

  if (std::filesystem::exists(p)) return p.string();

  // Original binary also contains the x86_64 path literally.
  p = project_root / "third_party" / "ch347" / "lib" /
      "x64" / "dynamic" / "libch347.so";

  return p.string();
}

// -----------------------------------------------------------------------------
// CH347 backend detection
// -----------------------------------------------------------------------------

static bool detect_ch347_backend(std::string &dev_path,
                                 std::string &lib_path) {
  const char *env_dev = std::getenv("SENSORD_CH347_DEV");
  const char *env_lib = std::getenv("SENSORD_CH347_LIB");

  if (env_dev && *env_dev) {
    dev_path = env_dev;
  } else {
    glob_t g{};
    if (glob("/dev/ch34x_pis*", 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
      dev_path = g.gl_pathv[0];
    }
    globfree(&g);

    if (dev_path.empty() &&
        glob("/dev/ttyACM*", 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
      dev_path = g.gl_pathv[0];
    }
    globfree(&g);
  }

  if (env_lib && *env_lib) {
    lib_path = env_lib;
  } else {
    lib_path = get_ch347_lib_path();
  }

  if (dev_path.empty()) {
    cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
               "detect_ch347_backend",
               "CH347 dev does not exist: %s", "/dev/ch34x_pis*");
    return false;
  }

  if (!std::filesystem::exists(lib_path)) {
    cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
               "detect_ch347_backend",
               "CH347 lib does not exist: %s", lib_path.c_str());
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// I2C bus detection
//
// The ELF confirms the exact function name, uint8_t& address parameter and
// strings for SENSORD_I2C_BUS, /dev/i2c-*, WHO_AM_I probing and fallback.
// -----------------------------------------------------------------------------

static int detect_imu_i2c_bus(uint8_t &addr) {
  const char *env = std::getenv("SENSORD_I2C_BUS");

  std::vector<int> buses;

  if (env && *env) {
    try {
      buses.push_back(std::stoi(env));
    } catch (...) {
      // Ignore malformed environment value and continue scanning.
    }
  } else {
    glob_t g{};
    if (glob("/dev/i2c-*", 0, nullptr, &g) == 0) {
      for (size_t i = 0; i < g.gl_pathc; ++i) {
        const char *p = g.gl_pathv[i];
        const char *dash = std::strrchr(p, '-');
        if (!dash) continue;
        try {
          buses.push_back(std::stoi(dash + 1));
        } catch (...) {}
      }
    }
    globfree(&g);
  }

  if (buses.empty()) buses.push_back(DEFAULT_I2C_BUS);

  std::sort(buses.begin(), buses.end());
  buses.erase(std::unique(buses.begin(), buses.end()), buses.end());

  for (int bus : buses) {
    cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
               "detect_imu_i2c_bus",
               "Probing IMU on env bus %d", bus);

    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);

    int fd = ::open(path, O_RDWR);
    if (fd < 0) continue;

    for (uint8_t a : {LSM6_ADDR_A, LSM6_ADDR_B}) {
      if (ioctl(fd, I2C_SLAVE_FORCE, a) < 0) continue;

      uint8_t reg = WHO_AM_I_REG;
      uint8_t who = 0;

      i2c_msg msgs[2]{};
      msgs[0].addr = a;
      msgs[0].flags = 0;
      msgs[0].len = 1;
      msgs[0].buf = &reg;

      msgs[1].addr = a;
      msgs[1].flags = I2C_M_RD;
      msgs[1].len = 1;
      msgs[1].buf = &who;

      i2c_rdwr_ioctl_data data{};
      data.msgs = msgs;
      data.nmsgs = 2;

      if (ioctl(fd, I2C_RDWR, &data) > 0 &&
          (who == LSM6_WHO_AM_I || who == LSM6TRC_WHO_AM_I)) {
        addr = a;
        ::close(fd);

        cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                   "detect_imu_i2c_bus",
                   "LSM6DS3 on /dev/i2c-%d, addr=0x%02X",
                   bus, addr);
        return bus;
      }
    }

    ::close(fd);
  }

  addr = LSM6_ADDR_A;
  cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
             "detect_imu_i2c_bus",
             "LSM6DS3 not found, defaulting to bus %d addr 0x%02X",
             DEFAULT_I2C_BUS, addr);
  return DEFAULT_I2C_BUS;
}

// -----------------------------------------------------------------------------
// Reinitialization helper
// -----------------------------------------------------------------------------

static bool reinit_sensor(LSM6DS3 &sensor, const std::string &name) {
  cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
             "reinit_sensor", "%s: reinitializing sensor...", name.c_str());

  try {
    sensor.shutdown();
    sensor.open();
    sensor.init_sensor();

    cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
               "reinit_sensor",
               "%s: sensor reinitialized successfully", name.c_str());
    return true;
  } catch (const std::exception &e) {
    cloudlog_e(ERROR, "system/sensord/ch347t.cc", 0,
               "reinit_sensor",
               "%s: reinitialization failed: %s",
               name.c_str(), e.what());
    return false;
  }
}

// -----------------------------------------------------------------------------
// Publishing helpers
//
// These match the binary's function names and line ordering.
// -----------------------------------------------------------------------------

static void publish_accelerometer(
    PubMaster &pm,
    cereal::SensorEventData::SensorSource source,
    float x, float y, float z) {

  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto data = evt.initAccelerometer();

  data.setSource(source);
  data.setVersion(1);
  data.setSensor(1);
  data.setType(1);
  data.setTimestamp(evt.getLogMonoTime());

  auto acc = data.initAcceleration();
  auto v = acc.initV(3);
  v.set(0, x);
  v.set(1, y);
  v.set(2, z);
  acc.setStatus(1);

  pm.send("accelerometer", msg);
}

static void publish_gyroscope(
    PubMaster &pm,
    cereal::SensorEventData::SensorSource source,
    float x, float y, float z) {

  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto data = evt.initGyroscope();

  data.setSource(source);
  data.setVersion(2);
  data.setSensor(5);
  data.setType(16);
  data.setTimestamp(evt.getLogMonoTime());

  auto gyro = data.initGyroUncalibrated();
  auto v = gyro.initV(3);
  v.set(0, x);
  v.set(1, y);
  v.set(2, z);
  gyro.setStatus(1);

  pm.send("gyroscope", msg);
}

static void publish_temperature(
    PubMaster &pm,
    cereal::SensorEventData::SensorSource source,
    float temp) {

  MessageBuilder msg;
  auto evt = msg.initEvent();
  auto data = evt.initTemperatureSensor();

  data.setSource(source);
  data.setTemperature(temp);

  pm.send("temperatureSensor", msg);
}

// -----------------------------------------------------------------------------
// Sensor cache
//
// The exact original type is compiler-generated in the binary. The decoded
// DWARF shows a local "cache" object and raw float fields. This structure
// represents the same information.
// -----------------------------------------------------------------------------

struct SensorCache {
  float acc[3] = {};
  float gyro[3] = {};
  float temperature = 0.0f;
  bool have_acc = false;
  bool have_gyro = false;
  bool have_temperature = false;
};

// -----------------------------------------------------------------------------
// Main sensor loop
//
// Recovered high-confidence behavior:
//
//   status = sensor.read_u8(0x1E)
//   bit 0 -> accelerometer block at 0x28, 6 bytes
//   bit 1 -> gyroscope block at 0x22, 6 bytes
//   signed little-endian int16 conversion
//   accel scale ~= 0.000598755
//   gyro scale  ~= 0.000152716
//   stale-data retry/reinit messages and backend switching
// -----------------------------------------------------------------------------

static void sensor_read_loop(
    LSM6DS3 &sensor,
    PubMaster &pm,
    RateKeeper &rk,
    const std::string &backend_name,
    std::atomic<bool> &switch_backend) {

  sensor.open();
  sensor.init_sensor();

  cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
             "sensor_read_loop", "Using %s backend",
             backend_name.c_str());

  SensorCache cache;
  int temp_ticks = 0;
  int consecutive_errors = 0;
  int reinit_attempts = 0;
  bool send_stale = false;
  uint64_t last_log_t = 0;
  float last_printed_acc[3] = {0};
  float last_printed_gyro[3] = {0};

  float yaw_offset = 0.0f;
  bool yaw_calibrated = false;
  std::vector<std::pair<float, float>> yaw_samples;

  std::string load_path = "/data/params/d/ImuYawOffset";
  std::ifstream yaw_file(load_path);
  if (!yaw_file.is_open()) {
    load_path = "/tmp/imu_yaw_offset";
    yaw_file.open(load_path);
  }
  if (yaw_file.is_open()) {
    if (yaw_file >> yaw_offset) {
      yaw_calibrated = true;
      cloudlog_e(INFO, "system/sensord/ch347t.cc", 0, "sensor_read_loop", "Loaded saved ImuYawOffset: %.4f rad", yaw_offset);
    }
    yaw_file.close();
  }

  while (!do_exit) {
    rk.keepTime();

    bool fresh_acc = false;
    bool fresh_gyro = false;

    uint8_t status = 0;

    try {
      // Binary calls virtual read_u8 with register 0x1E.
      status = sensor.read_u8(0x1E);

      if (status & 0x01) {
        auto b = sensor.read_block(0x28, 6);
        if (b.size() >= 6) {
          int16_t x = static_cast<int16_t>(
              static_cast<uint16_t>(b[0]) |
              (static_cast<uint16_t>(b[1]) << 8));
          int16_t y = static_cast<int16_t>(
              static_cast<uint16_t>(b[2]) |
              (static_cast<uint16_t>(b[3]) << 8));
          int16_t z = static_cast<int16_t>(
              static_cast<uint16_t>(b[4]) |
              (static_cast<uint16_t>(b[5]) << 8));

          cache.acc[0] = -y * ACCEL_SCALE;
          cache.acc[1] = x * ACCEL_SCALE;
          cache.acc[2] = z * ACCEL_SCALE;
          cache.have_acc = true;
          fresh_acc = true;
        }
      }

      if (status & 0x02) {
        auto b = sensor.read_block(0x22, 6);
        if (b.size() >= 6) {
          int16_t x = static_cast<int16_t>(
              static_cast<uint16_t>(b[0]) |
              (static_cast<uint16_t>(b[1]) << 8));
          int16_t y = static_cast<int16_t>(
              static_cast<uint16_t>(b[2]) |
              (static_cast<uint16_t>(b[3]) << 8));
          int16_t z = static_cast<int16_t>(
              static_cast<uint16_t>(b[4]) |
              (static_cast<uint16_t>(b[5]) << 8));

          cache.gyro[0] = -y * GYRO_SCALE;
          cache.gyro[1] = x * GYRO_SCALE;
          cache.gyro[2] = z * GYRO_SCALE;
          cache.have_gyro = true;
          fresh_gyro = true;
        }
      }

      // Temperature output is at 0x20/0x21 on LSM6DS3.
      // The original binary has temp_ticks and publish_temperature().
      try {
        auto b = sensor.read_block(0x20, 2);
        if (b.size() >= 2) {
          temp_ticks = static_cast<int16_t>(
              static_cast<uint16_t>(b[0]) |
              (static_cast<uint16_t>(b[1]) << 8));
          cache.temperature = TEMP_OFFSET + temp_ticks * TEMP_SCALE;
          cache.have_temperature = true;
        }
      } catch (...) {
        // Temperature is non-critical.
      }

      if (consecutive_errors > 0) {
        fprintf(stderr, "system/sensord/ch347t.cc: %s: I2C recovered after %d errors\n",
                backend_name.c_str(), consecutive_errors);
        fflush(stderr);
      }

      consecutive_errors = 0;
      reinit_attempts = 0;

    } catch (const std::exception &e) {
      ++consecutive_errors;

      if (cache.have_acc || cache.have_gyro) {
        send_stale = true;

        cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
                   "sensor_read_loop",
                   "%s: I2C error, sending stale data (%d/%d)",
                   backend_name.c_str(),
                   consecutive_errors,
                   REINIT_ERROR_THRESHOLD);

        fprintf(stderr, "system/sensord/ch347t.cc: %s: I2C error, sending stale data (%d/%d)\n",
                backend_name.c_str(), consecutive_errors, REINIT_ERROR_THRESHOLD);
        fflush(stderr);
      }

      if (consecutive_errors >= REINIT_ERROR_THRESHOLD) {
        if (reinit_attempts < MAX_REINIT_ATTEMPTS) {
          ++reinit_attempts;

          cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
                     "sensor_read_loop",
                     "%s: %d consecutive errors, reinit (%d/%d)",
                     backend_name.c_str(),
                     consecutive_errors,
                     reinit_attempts,
                     MAX_REINIT_ATTEMPTS);

          if (reinit_sensor(sensor, backend_name)) {
            consecutive_errors = 0;
            continue;
          }
        } else {
          cloudlog_e(ERROR, "system/sensord/ch347t.cc", 0,
                     "sensor_read_loop",
                     "%s: reinit failed after %d attempts, giving up",
                     backend_name.c_str(), reinit_attempts);

          switch_backend.store(true);
          break;
        }
      }

      usleep(2000);
    }

    // 自动 XY 偏航角自适应标定
    if (!yaw_calibrated && cache.have_acc && cache.have_gyro && fresh_acc && fresh_gyro) {
      float gz = cache.gyro[2];
      float ax = cache.acc[0];
      float ay = cache.acc[1];
      float norm = std::sqrt(ax * ax + ay * ay);

      if (std::fabs(gz) < 0.02f && norm > 1.0f && norm < 4.0f) {
        yaw_samples.push_back({ax, ay});
        if (yaw_samples.size() >= 150) {
          float sum_x = 0, sum_y = 0;
          for (const auto& s : yaw_samples) {
            sum_x += s.first;
            sum_y += s.second;
          }
          float avg_x = sum_x / yaw_samples.size();
          float avg_y = sum_y / yaw_samples.size();
          yaw_offset = std::atan2(-avg_y, -avg_x);
          yaw_calibrated = true;

          std::string save_path = "/data/params/d/ImuYawOffset";
          std::ofstream out_file(save_path);
          if (!out_file.is_open()) {
            save_path = "/tmp/imu_yaw_offset";
            out_file.open(save_path);
          }
          if (out_file.is_open()) {
            out_file << yaw_offset;
            out_file.close();
          }
          cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0, "sensor_read_loop",
                     "IMU Yaw auto-calibrated: %.4f rad (%.2f deg)", yaw_offset, yaw_offset * 180.0f / M_PI);
        }
      }
    }

    // 应用自适应 XY 旋转
    if (yaw_calibrated) {
      float cos_y = std::cos(yaw_offset);
      float sin_y = std::sin(yaw_offset);

      if (cache.have_acc && (fresh_acc || send_stale)) {
        float rx = cache.acc[0];
        float ry = cache.acc[1];
        cache.acc[0] = rx * cos_y - ry * sin_y;
        cache.acc[1] = rx * sin_y + ry * cos_y;
      }

      if (cache.have_gyro && (fresh_gyro || send_stale)) {
        float gx = cache.gyro[0];
        float gy = cache.gyro[1];
        cache.gyro[0] = gx * cos_y - gy * sin_y;
        cache.gyro[1] = gx * sin_y + gy * cos_y;
      }
    }

    const auto source = sensor.source();

    if (cache.have_acc && (fresh_acc || send_stale)) {
      publish_accelerometer(pm, source,
                            cache.acc[0],
                            cache.acc[1],
                            cache.acc[2]);

      cloudlog_e(DEBUG, "system/sensord/ch347t.cc", 0,
                 "sensor_read_loop",
                 "[%s] ACC: [%7.2f, %7.2f, %7.2f] m/s",
                 backend_name.c_str(),
                 cache.acc[0], cache.acc[1], cache.acc[2]);
    }

    if (cache.have_gyro && (fresh_gyro || send_stale)) {
      publish_gyroscope(pm, source,
                        cache.gyro[0],
                        cache.gyro[1],
                        cache.gyro[2]);

      cloudlog_e(DEBUG, "system/sensord/ch347t.cc", 0,
                 "sensor_read_loop",
                 "[%s] GYRO: [%7.3f, %7.3f, %7.3f] rad/s",
                 backend_name.c_str(),
                 cache.gyro[0], cache.gyro[1], cache.gyro[2]);
    }

    if (cache.have_temperature) {
      publish_temperature(pm, source, cache.temperature);
    }

    send_stale = false;

    uint64_t now = nanos_since_boot();
    float diff_acc = 0.0f;
    float diff_gyro = 0.0f;
    if (cache.have_acc) {
      diff_acc = std::max({std::fabs(cache.acc[0] - last_printed_acc[0]),
                           std::fabs(cache.acc[1] - last_printed_acc[1]),
                           std::fabs(cache.acc[2] - last_printed_acc[2])});
    }
    if (cache.have_gyro) {
      diff_gyro = std::max({std::fabs(cache.gyro[0] - last_printed_gyro[0]),
                            std::fabs(cache.gyro[1] - last_printed_gyro[1]),
                            std::fabs(cache.gyro[2] - last_printed_gyro[2])});
    }

    bool big_change = (diff_acc > 0.5f) || (diff_gyro > 0.05f);
    uint64_t print_interval = big_change ? 1e9 : 10e9; // 变化大时1s，变化小时10s

    if (now - last_log_t > print_interval) {
      if (cache.have_acc && cache.have_gyro) {
        fprintf(stderr, "[ch347t] ACC: [%9.4f, %9.4f, %9.4f] m/s2\n",
                cache.acc[0], cache.acc[1], cache.acc[2]);
        fprintf(stderr, "[ch347t] GYRO: [%9.5f, %9.5f, %9.5f] rad/s\n",
                cache.gyro[0], cache.gyro[1], cache.gyro[2]);
        fflush(stderr);
      }
      last_log_t = now;
      if (cache.have_acc) {
        memcpy(last_printed_acc, cache.acc, sizeof(last_printed_acc));
      }
      if (cache.have_gyro) {
        memcpy(last_printed_gyro, cache.gyro, sizeof(last_printed_gyro));
      }
    }
  }

  try {
    sensor.shutdown();
  } catch (...) {}
}

// -----------------------------------------------------------------------------
// CH347 waiting helper
//
// DWARF says:
//   wait_for_ch347(std::string&, std::string&)
//   declaration around line 750
//   return bool
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// main
//
// The binary's main contains additional machine-license and hardware-detection
// code. The following preserves the observed backend state machine while
// deliberately keeping that unrelated policy code out of the reconstruction.
// -----------------------------------------------------------------------------

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  if (std::getenv("SIMULATION")) {
    // Original binary contains a SIMULATION static symbol.
  }

  PubMaster pm({"accelerometer", "gyroscope", "temperatureSensor"});
  RateKeeper rk("sensord_ch347", 104.0f);

  std::string ch347_dev;
  std::string ch347_lib;

  std::atomic<bool> switch_backend{false};

  while (!do_exit) {
    bool have_ch347 = detect_ch347_backend(ch347_dev, ch347_lib);

    if (have_ch347) {
      cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                 "main", "CH347 backend available, starting...");

      void *handle = dlopen(ch347_lib.c_str(), RTLD_NOW | RTLD_LOCAL);
      if (!handle) {
        cloudlog_e(ERROR, "system/sensord/ch347t.cc", 0,
                   "main", "Failed to load CH347 library: %s",
                   dlerror());
      } else {
        try {
          CH347LSM6 sensor(ch347_dev, handle);
          switch_backend.store(false);

          sensor_read_loop(sensor, pm, rk, "CH347", switch_backend);

          dlclose(handle);

          if (!switch_backend) {
            cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                       "main", "CH347: clean exit");
            break;
          }

          cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
                     "main",
                     "CH347 backend failed, falling back to direct I2C");
        } catch (const std::exception &e) {
          cloudlog_e(ERROR, "system/sensord/ch347t.cc", 0,
                     "main", "CH347 backend failed: %s", e.what());
          dlclose(handle);
        }
      }
    } else {
      cloudlog_e(WARNING, "system/sensord/ch347t.cc", 0,
                 "main", "CH347 not available after waiting");
    }

    uint8_t addr = LSM6_ADDR_A;
    int bus = detect_imu_i2c_bus(addr);

    try {
      I2CLSM6 sensor(bus, addr);
      cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                 "main",
                 "Starting direct I2C backend (bus=%d, addr=0x%02X)",
                 bus, addr);

      switch_backend.store(false);
      sensor_read_loop(sensor, pm, rk, "I2C", switch_backend);

      if (!switch_backend) {
        cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                   "main", "I2C: clean exit");
        break;
      }

    } catch (const std::exception &e) {
      cloudlog_e(ERROR, "system/sensord/ch347t.cc", 0,
                 "main",
                 "I2C backend failed, retrying in 2s...");
      sleep(2);
    }

    if (switch_backend && detect_ch347_backend(ch347_dev, ch347_lib)) {
      cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
                 "main", "Switching from I2C to CH347...");
    }
  }

  cloudlog_e(INFO, "system/sensord/ch347t.cc", 0,
             "main", "sensord_ch347 exiting");

  return 0;
}
