#pragma once

#include <eigen3/Eigen/Dense>
#include <deque>
#include <fstream>
#include <memory>
#include <map>
#include <string>
#include <thread>

#include "cereal/messaging/messaging.h"
#include "common/transformations/coordinates.hpp"
#include "common/transformations/orientation.hpp"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#include "system/sensord/sensors/constants.h"
#define VISION_DECIMATION 2
#define SENSOR_DECIMATION 10
#include "selfdrive/locationd/models/live_kf.h"

#define POSENET_STD_HIST_HALF 20

enum LocalizerGnssSource {
  UBLOX, QCOM
};

// JY62设备类型枚举
enum class ImuDeviceType {
  NONE,
  JY62
};

class Localizer {
public:
  Localizer(LocalizerGnssSource gnss_source = LocalizerGnssSource::UBLOX);

  int locationd_thread();

  void reset_kalman(double current_time = NAN);
  void reset_kalman(double current_time, const Eigen::VectorXd &init_orient, const Eigen::VectorXd &init_pos, const Eigen::VectorXd &init_vel, const MatrixXdr &init_pos_R, const MatrixXdr &init_vel_R);
  void reset_kalman(double current_time, const Eigen::VectorXd &init_x, const MatrixXdr &init_P);
  void finite_check(double current_time = NAN);
  void time_check(double current_time = NAN);
  void update_reset_tracker();
  bool is_gps_ok();
  bool critical_services_valid(const std::map<std::string, double> &critical_services);
  bool is_timestamp_valid(double current_time);
  void determine_gps_mode(double current_time);
  bool are_inputs_ok();
  void observation_timings_invalid_reset();

  kj::ArrayPtr<capnp::byte> get_message_bytes(MessageBuilder& msg_builder,
    bool inputsOK, bool sensorsOK, bool gpsOK, bool msgValid);
  void build_live_location(cereal::LiveLocationKalman::Builder& fix);

  Eigen::VectorXd get_position_geodetic();
  Eigen::VectorXd get_state();
  Eigen::VectorXd get_stdev();

  void handle_msg_bytes(const char *data, const size_t size);
  void handle_msg(const cereal::Event::Reader& log);
  void handle_sensor(double current_time, const cereal::SensorEventData::Reader& log);
  void handle_gps(double current_time, const cereal::GpsLocationData::Reader& log, const double sensor_time_offset);
  void handle_gnss(double current_time, const cereal::GnssMeasurements::Reader& log);
  void handle_car_state(double current_time, const cereal::CarState::Reader& log);
  void handle_cam_odo(double current_time, const cereal::CameraOdometry::Reader& log);
  void handle_live_calib(double current_time, const cereal::LiveCalibrationData::Reader& log);

  void input_fake_gps_observations(double current_time);

  // JY62设备相关的方法声明
  void set_device_type(ImuDeviceType type);
  bool is_jy62() const;
  void set_device_params(const std::string& device_path, int baud_rate);
  std::tuple<double, double, double> parse_accelerometer(const std::string& line);
  std::tuple<double, double, double> parse_gyroscope(const std::string& line);
  std::tuple<double, double, double> parse_angle(const std::string& line);

private:
  std::unique_ptr<LiveKalman> kf;

  Eigen::VectorXd calib;
  MatrixXdr device_from_calib;
  MatrixXdr calib_from_device;
  bool calibrated = false;

  double car_speed = 0.0;
  double last_reset_time = NAN;
  std::deque<double> posenet_stds;

  std::unique_ptr<LocalCoord> converter;

  int64_t unix_timestamp_millis = 0;
  double reset_tracker = 0.0;
  bool device_fell = false;
  bool gps_mode = false;
  double first_valid_log_time = NAN;
  double ttff = NAN;
  double last_gps_msg = 0;
  bool observation_timings_invalid = false;
  std::map<std::string, double> observation_values_invalid;
  bool standstill = true;
  int32_t orientation_reset_count = 0;
  float gps_std_factor;
  float gps_variance_factor;
  float gps_vertical_variance_factor;
  double gps_time_offset;
  Eigen::VectorXd camodo_yawrate_distribution = Eigen::Vector2d(0.0, 10.0); // mean, std

  uint32_t accel_data_cnt = 0;
  uint32_t gyro_data_cnt = 0;
  std::unique_ptr<PubMaster> _pm;  // 添加这一行

  // JY62设备相关的成员变量
  ImuDeviceType device_type_;
  bool is_jy62_device_ = false;
  std::string device_path_;
  int baud_rate_;
  int jy62_fd_;
  std::thread jy62_thread_;
  bool jy62_running_ = false;

  // JY62设备私有方法
  int open_jy62_device();
  std::string read_jy62_line();
  void publish_accelerometer(double x, double y, double z);
  void publish_gyroscope(double x, double y, double z);
  void publish_orientation(double pitch, double roll, double yaw);
  std::vector<uint8_t> read_jy62_packet();
  bool parse_jy62_packet(const std::vector<uint8_t>& packet,
                         double& accel_x, double& accel_y, double& accel_z,
                         double& gyro_x, double& gyro_y, double& gyro_z);
  void jy62_reader_thread();
  void start_jy62_reader();
  void stop_jy62_reader();

};