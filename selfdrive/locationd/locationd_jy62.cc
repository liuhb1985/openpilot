#include <getopt.h>

#include "selfdrive/locationd/locationd_jy62.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <algorithm>
#include <iterator>

using namespace EKFS;
using namespace Eigen;

ExitHandler do_exit;
const double ACCEL_SANITY_CHECK = 100.0;  // m/s^2
const double ROTATION_SANITY_CHECK = 10.0;  // rad/s
const double TRANS_SANITY_CHECK = 200.0;  // m/s
const double CALIB_RPY_SANITY_CHECK = 0.5; // rad (+- 30 deg)
const double ALTITUDE_SANITY_CHECK = 10000; // m
const double MIN_STD_SANITY_CHECK = 1e-5; // m or rad
const double VALID_TIME_SINCE_RESET = 1.0; // s
const double VALID_POS_STD = 50.0; // m
const double MAX_RESET_TRACKER = 5.0;
const double SANE_GPS_UNCERTAINTY = 1500.0; // m
const double INPUT_INVALID_THRESHOLD = 0.5; // same as reset tracker
const double RESET_TRACKER_DECAY = 0.99995;
const double DECAY = 0.9993; // ~10 secs to resume after a bad input
const double MAX_FILTER_REWIND_TIME = 0.8; // s
const double YAWRATE_CROSS_ERR_CHECK_FACTOR = 30;

// TODO: GPS sensor time offsets are empirically calculated
// They should be replaced with synced time from a real clock
const double GPS_QUECTEL_SENSOR_TIME_OFFSET = 0.630; // s
const double GPS_UBLOX_SENSOR_TIME_OFFSET = 0.095; // s
const float  GPS_POS_STD_THRESHOLD = 50.0;
const float  GPS_VEL_STD_THRESHOLD = 5.0;
const float  GPS_POS_ERROR_RESET_THRESHOLD = 300.0;
const float  GPS_POS_STD_RESET_THRESHOLD = 2.0;
const float  GPS_VEL_STD_RESET_THRESHOLD = 0.5;
const float  GPS_ORIENTATION_ERROR_RESET_THRESHOLD = 1.0;
const int    GPS_ORIENTATION_ERROR_RESET_CNT = 3;

const bool   DEBUG = getenv("DEBUG") != nullptr && std::string(getenv("DEBUG")) != "0";

static VectorXd floatlist2vector(const capnp::List<float, capnp::Kind::PRIMITIVE>::Reader& floatlist) {
  VectorXd res(floatlist.size());
  for (int i = 0; i < floatlist.size(); i++) {
    res[i] = floatlist[i];
  }
  return res;
}

static Vector4d quat2vector(const Quaterniond& quat) {
  return Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
}

static Quaterniond vector2quat(const VectorXd& vec) {
  return Quaterniond(vec(0), vec(1), vec(2), vec(3));
}

static void init_measurement(cereal::LiveLocationKalman::Measurement::Builder meas, const VectorXd& val, const VectorXd& std, bool valid) {
  meas.setValue(kj::arrayPtr(val.data(), val.size()));
  meas.setStd(kj::arrayPtr(std.data(), std.size()));
  meas.setValid(valid);
}


static MatrixXdr rotate_cov(const MatrixXdr& rot_matrix, const MatrixXdr& cov_in) {
  // To rotate a covariance matrix, the cov matrix needs to multiplied left and right by the transform matrix
  return ((rot_matrix *  cov_in) * rot_matrix.transpose());
}

static VectorXd rotate_std(const MatrixXdr& rot_matrix, const VectorXd& std_in) {
  // Stds cannot be rotated like values, only covariances can be rotated
  return rotate_cov(rot_matrix, std_in.array().square().matrix().asDiagonal()).diagonal().array().sqrt();
}

Localizer::Localizer(LocalizerGnssSource gnss_source) {
  this->kf = std::make_unique<LiveKalman>();
  this->reset_kalman();

  this->calib = Vector3d(0.0, 0.0, 0.0);
  this->device_from_calib = MatrixXdr::Identity(3, 3);
  this->calib_from_device = MatrixXdr::Identity(3, 3);

  for (int i = 0; i < POSENET_STD_HIST_HALF * 2; i++) {
    this->posenet_stds.push_back(10.0);
  }

  VectorXd ecef_pos = this->kf->get_x().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
  this->converter = std::make_unique<LocalCoord>((ECEF) { .x = ecef_pos[0], .y = ecef_pos[1], .z = ecef_pos[2] });

  // 初始化设备类型
  this->device_type_ = ImuDeviceType::NONE;
  this->is_jy62_device_ = false;
  this->device_path_ = "/dev/ttyUSB0";
  this->baud_rate_ = 115200;  // JY62设备默认波特率
  this->jy62_fd_ = -1;
  this->jy62_running_ = false;

  _pm = std::make_unique<PubMaster>(std::vector<const char*>{"accelerometer", "gyroscope"});
}

void Localizer::build_live_location(cereal::LiveLocationKalman::Builder& fix) {
  VectorXd predicted_state = this->kf->get_x();
  MatrixXdr predicted_cov = this->kf->get_P();
  VectorXd predicted_std = predicted_cov.diagonal().array().sqrt();

  VectorXd fix_ecef = predicted_state.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
  ECEF fix_ecef_ecef = { .x = fix_ecef(0), .y = fix_ecef(1), .z = fix_ecef(2) };
  VectorXd fix_ecef_std = predicted_std.segment<STATE_ECEF_POS_ERR_LEN>(STATE_ECEF_POS_ERR_START);
  VectorXd vel_ecef = predicted_state.segment<STATE_ECEF_VELOCITY_LEN>(STATE_ECEF_VELOCITY_START);
  VectorXd vel_ecef_std = predicted_std.segment<STATE_ECEF_VELOCITY_ERR_LEN>(STATE_ECEF_VELOCITY_ERR_START);
  VectorXd fix_pos_geo_vec = this->get_position_geodetic();
  VectorXd orientation_ecef = quat2euler(vector2quat(predicted_state.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START)));
  VectorXd orientation_ecef_std = predicted_std.segment<STATE_ECEF_ORIENTATION_ERR_LEN>(STATE_ECEF_ORIENTATION_ERR_START);
  MatrixXdr orientation_ecef_cov = predicted_cov.block<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>(STATE_ECEF_ORIENTATION_ERR_START, STATE_ECEF_ORIENTATION_ERR_START);
  MatrixXdr device_from_ecef = euler2rot(orientation_ecef).transpose();
  VectorXd calibrated_orientation_ecef = rot2euler((this->calib_from_device * device_from_ecef).transpose());

  VectorXd acc_calib = this->calib_from_device * predicted_state.segment<STATE_ACCELERATION_LEN>(STATE_ACCELERATION_START);
  MatrixXdr acc_calib_cov = predicted_cov.block<STATE_ACCELERATION_ERR_LEN, STATE_ACCELERATION_ERR_LEN>(STATE_ACCELERATION_ERR_START, STATE_ACCELERATION_ERR_START);
  VectorXd acc_calib_std = rotate_cov(this->calib_from_device, acc_calib_cov).diagonal().array().sqrt();
  VectorXd ang_vel_calib = this->calib_from_device * predicted_state.segment<STATE_ANGULAR_VELOCITY_LEN>(STATE_ANGULAR_VELOCITY_START);

  MatrixXdr vel_angular_cov = predicted_cov.block<STATE_ANGULAR_VELOCITY_ERR_LEN, STATE_ANGULAR_VELOCITY_ERR_LEN>(STATE_ANGULAR_VELOCITY_ERR_START, STATE_ANGULAR_VELOCITY_ERR_START);
  VectorXd ang_vel_calib_std = rotate_cov(this->calib_from_device, vel_angular_cov).diagonal().array().sqrt();

  VectorXd vel_device = device_from_ecef * vel_ecef;
  VectorXd device_from_ecef_eul = quat2euler(vector2quat(predicted_state.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START))).transpose();
  MatrixXdr condensed_cov(STATE_ECEF_ORIENTATION_ERR_LEN + STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN + STATE_ECEF_VELOCITY_ERR_LEN);
  condensed_cov.topLeftCorner<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>() =
    predicted_cov.block<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>(STATE_ECEF_ORIENTATION_ERR_START, STATE_ECEF_ORIENTATION_ERR_START);
  condensed_cov.topRightCorner<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_VELOCITY_ERR_LEN>() =
    predicted_cov.block<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_VELOCITY_ERR_LEN>(STATE_ECEF_ORIENTATION_ERR_START, STATE_ECEF_VELOCITY_ERR_START);
  condensed_cov.bottomRightCorner<STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_VELOCITY_ERR_LEN>() =
    predicted_cov.block<STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_VELOCITY_ERR_LEN>(STATE_ECEF_VELOCITY_ERR_START, STATE_ECEF_VELOCITY_ERR_START);
  condensed_cov.bottomLeftCorner<STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>() =
    predicted_cov.block<STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>(STATE_ECEF_VELOCITY_ERR_START, STATE_ECEF_ORIENTATION_ERR_START);
  VectorXd H_input(device_from_ecef_eul.size() + vel_ecef.size());
  H_input << device_from_ecef_eul, vel_ecef;
  MatrixXdr HH = this->kf->H(H_input);
  MatrixXdr vel_device_cov = (HH * condensed_cov) * HH.transpose();
  VectorXd vel_device_std = vel_device_cov.diagonal().array().sqrt();

  VectorXd vel_calib = this->calib_from_device * vel_device;
  VectorXd vel_calib_std = rotate_cov(this->calib_from_device, vel_device_cov).diagonal().array().sqrt();

  VectorXd orientation_ned = ned_euler_from_ecef(fix_ecef_ecef, orientation_ecef);
  VectorXd orientation_ned_std = rotate_cov(this->converter->ecef2ned_matrix, orientation_ecef_cov).diagonal().array().sqrt();
  VectorXd calibrated_orientation_ned = ned_euler_from_ecef(fix_ecef_ecef, calibrated_orientation_ecef);
  VectorXd nextfix_ecef = fix_ecef + vel_ecef;
  VectorXd ned_vel = this->converter->ecef2ned((ECEF) { .x = nextfix_ecef(0), .y = nextfix_ecef(1), .z = nextfix_ecef(2) }).to_vector() - converter->ecef2ned(fix_ecef_ecef).to_vector();

  VectorXd accDevice = predicted_state.segment<STATE_ACCELERATION_LEN>(STATE_ACCELERATION_START);
  VectorXd accDeviceErr = predicted_std.segment<STATE_ACCELERATION_ERR_LEN>(STATE_ACCELERATION_ERR_START);

  VectorXd angVelocityDevice = predicted_state.segment<STATE_ANGULAR_VELOCITY_LEN>(STATE_ANGULAR_VELOCITY_START);
  VectorXd angVelocityDeviceErr = predicted_std.segment<STATE_ANGULAR_VELOCITY_ERR_LEN>(STATE_ANGULAR_VELOCITY_ERR_START);

  Vector3d nans = Vector3d(NAN, NAN, NAN);

  // TODO fill in NED and Calibrated stds
  // write measurements to msg
  init_measurement(fix.initPositionGeodetic(), fix_pos_geo_vec, nans, this->gps_mode);
  init_measurement(fix.initPositionECEF(), fix_ecef, fix_ecef_std, this->gps_mode);
  init_measurement(fix.initVelocityECEF(), vel_ecef, vel_ecef_std, this->gps_mode);
  init_measurement(fix.initVelocityNED(), ned_vel, nans, this->gps_mode);
  init_measurement(fix.initVelocityDevice(), vel_device, vel_device_std, true);
  init_measurement(fix.initAccelerationDevice(), accDevice, accDeviceErr, true);
  init_measurement(fix.initOrientationECEF(), orientation_ecef, orientation_ecef_std, this->gps_mode);
  init_measurement(fix.initCalibratedOrientationECEF(), calibrated_orientation_ecef, nans, this->calibrated && this->gps_mode);
  init_measurement(fix.initOrientationNED(), orientation_ned, orientation_ned_std, this->gps_mode);
  init_measurement(fix.initCalibratedOrientationNED(), calibrated_orientation_ned, nans, this->calibrated && this->gps_mode);
  init_measurement(fix.initAngularVelocityDevice(), angVelocityDevice, angVelocityDeviceErr, true);
  init_measurement(fix.initVelocityCalibrated(), vel_calib, vel_calib_std, this->calibrated);
  init_measurement(fix.initAngularVelocityCalibrated(), ang_vel_calib, ang_vel_calib_std, this->calibrated);
  init_measurement(fix.initAccelerationCalibrated(), acc_calib, acc_calib_std, this->calibrated);
  if (DEBUG) {
    init_measurement(fix.initFilterState(), predicted_state, predicted_std, true);
  }

  double old_mean = 0.0, new_mean = 0.0;
  int i = 0;
  for (double x : this->posenet_stds) {
    if (i < POSENET_STD_HIST_HALF) {
      old_mean += x;
    } else {
      new_mean += x;
    }
    i++;
  }
  old_mean /= POSENET_STD_HIST_HALF;
  new_mean /= POSENET_STD_HIST_HALF;
  // experimentally found these values, no false positives in 20k minutes of driving
  bool std_spike = (new_mean / old_mean > 4.0 && new_mean > 7.0);

  fix.setPosenetOK(!(std_spike && this->car_speed > 5.0));
  fix.setDeviceStable(!this->device_fell);
  fix.setExcessiveResets(this->reset_tracker > MAX_RESET_TRACKER);
  fix.setTimeToFirstFix(std::isnan(this->ttff) ? -1. : this->ttff);
  this->device_fell = false;

  //fix.setGpsWeek(this->time.week);
  //fix.setGpsTimeOfWeek(this->time.tow);
  fix.setUnixTimestampMillis(this->unix_timestamp_millis);

  double time_since_reset = this->kf->get_filter_time() - this->last_reset_time;
  fix.setTimeSinceReset(time_since_reset);
  if (fix_ecef_std.norm() < VALID_POS_STD && this->calibrated && time_since_reset > VALID_TIME_SINCE_RESET) {
    fix.setStatus(cereal::LiveLocationKalman::Status::VALID);
  } else if (fix_ecef_std.norm() < VALID_POS_STD && time_since_reset > VALID_TIME_SINCE_RESET) {
    fix.setStatus(cereal::LiveLocationKalman::Status::UNCALIBRATED);
  } else {
    fix.setStatus(cereal::LiveLocationKalman::Status::UNINITIALIZED);
  }
}

VectorXd Localizer::get_position_geodetic() {
  VectorXd fix_ecef = this->kf->get_x().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
  ECEF fix_ecef_ecef = { .x = fix_ecef(0), .y = fix_ecef(1), .z = fix_ecef(2) };
  Geodetic fix_pos_geo = ecef2geodetic(fix_ecef_ecef);
  return Vector3d(fix_pos_geo.lat, fix_pos_geo.lon, fix_pos_geo.alt);
}

VectorXd Localizer::get_state() {
  return this->kf->get_x();
}

VectorXd Localizer::get_stdev() {
  return this->kf->get_P().diagonal().array().sqrt();
}

bool Localizer::are_inputs_ok() {
  return this->critical_services_valid(this->observation_values_invalid) && !this->observation_timings_invalid;
}

void Localizer::observation_timings_invalid_reset(){
  this->observation_timings_invalid = false;
}

void Localizer::handle_sensor(double current_time, const cereal::SensorEventData::Reader& log) {
  // TODO does not yet account for double sensor readings in the log

  // Ignore empty readings (e.g. in case the magnetometer had no data ready)
  if (log.getTimestamp() == 0) {
    return;
  }

  double sensor_time = 1e-9 * log.getTimestamp();

  // sensor time and log time should be close
  if (std::abs(current_time - sensor_time) > 0.1) {
    LOGE("Sensor reading ignored, sensor timestamp more than 100ms off from log time");
    this->observation_timings_invalid = true;
    return;
  } else if (!this->is_timestamp_valid(sensor_time)) {
    this->observation_timings_invalid = true;
    return;
  }

  // TODO: handle messages from two IMUs at the same time
  // For JY62 device, we allow BMX055 source
  if (log.getSource() == cereal::SensorEventData::SensorSource::BMX055 && !this->is_jy62()) {
    return;
  }

  // Gyro Uncalibrated
  if (log.getSensor() == SENSOR_GYRO_UNCALIBRATED && log.getType() == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
    auto v = log.getGyroUncalibrated().getV();
    auto meas = Vector3d(-v[2], -v[1], -v[0]);

    if((0 == (this->gyro_data_cnt % 10)) && (this->gyro_data_cnt < 1000)){
      printf("[%u]Gyro: v[0]=%.3f,v[1]=%.3f,v[2]=%.3f\n", this->gyro_data_cnt,v[0],v[1],v[2]);
    }
    this->gyro_data_cnt++;

    VectorXd gyro_bias = this->kf->get_x().segment<STATE_GYRO_BIAS_LEN>(STATE_GYRO_BIAS_START);
    float gyro_camodo_yawrate_err = std::abs((meas[2] - gyro_bias[2]) - this->camodo_yawrate_distribution[0]);
    float gyro_camodo_yawrate_err_threshold = YAWRATE_CROSS_ERR_CHECK_FACTOR * this->camodo_yawrate_distribution[1];
    bool gyro_valid = gyro_camodo_yawrate_err < gyro_camodo_yawrate_err_threshold;

    if ((meas.norm() < ROTATION_SANITY_CHECK) && gyro_valid) {
      this->kf->predict_and_observe(sensor_time, OBSERVATION_PHONE_GYRO, { meas });
      this->observation_values_invalid["gyroscope"] *= DECAY;
    } else {
      this->observation_values_invalid["gyroscope"] += 1.0;
    }
  }

  // Accelerometer
  if (log.getSensor() == SENSOR_ACCELEROMETER && log.getType() == SENSOR_TYPE_ACCELEROMETER) {
    auto v = log.getAcceleration().getV();

    // TODO: reduce false positives and re-enable this check
    // check if device fell, estimate 10 for g
    // 40m/s**2 is a good filter for falling detection, no false positives in 20k minutes of driving
    // this->device_fell |= (floatlist2vector(v) - Vector3d(10.0, 0.0, 0.0)).norm() > 40.0;

    if((0 == (this->accel_data_cnt % 10)) && (this->accel_data_cnt < 1000)){
      printf("[%u]Accel: v[0]=%.3f,v[1]=%.3f,v[2]=%.3f\n", this->accel_data_cnt,v[0],v[1],v[2]);
    }
    this->accel_data_cnt++;

    auto meas = Vector3d(-v[2], -v[1], -v[0]);
    if (meas.norm() < ACCEL_SANITY_CHECK) {
      this->kf->predict_and_observe(sensor_time, OBSERVATION_PHONE_ACCEL, { meas });
      this->observation_values_invalid["accelerometer"] *= DECAY;
    } else {
      this->observation_values_invalid["accelerometer"] += 1.0;
    }
  }
}

void Localizer::input_fake_gps_observations(double current_time) {
  // This is done to make sure that the error estimate of the position does not blow up
  // when the filter is in no-gps mode
  // Steps : first predict -> observe current obs with reasonable STD
  this->kf->predict(current_time);

  VectorXd current_x = this->kf->get_x();
  VectorXd ecef_pos = current_x.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
  VectorXd ecef_vel = current_x.segment<STATE_ECEF_VELOCITY_LEN>(STATE_ECEF_VELOCITY_START);
  const MatrixXdr &ecef_pos_R = this->kf->get_fake_gps_pos_cov();
  const MatrixXdr &ecef_vel_R = this->kf->get_fake_gps_vel_cov();

  this->kf->predict_and_observe(current_time, OBSERVATION_ECEF_POS, { ecef_pos }, { ecef_pos_R });
  this->kf->predict_and_observe(current_time, OBSERVATION_ECEF_VEL, { ecef_vel }, { ecef_vel_R });
}

void Localizer::handle_gps(double current_time, const cereal::GpsLocationData::Reader& log, const double sensor_time_offset) {
  bool gps_unreasonable = (Vector2d(log.getHorizontalAccuracy(), log.getVerticalAccuracy()).norm() >= SANE_GPS_UNCERTAINTY);
  bool gps_accuracy_insane = ((log.getVerticalAccuracy() <= 0) || (log.getSpeedAccuracy() <= 0) || (log.getBearingAccuracyDeg() <= 0));
  bool gps_lat_lng_alt_insane = ((std::abs(log.getLatitude()) > 90) || (std::abs(log.getLongitude()) > 180) || (std::abs(log.getAltitude()) > ALTITUDE_SANITY_CHECK));
  bool gps_vel_insane = (floatlist2vector(log.getVNED()).norm() > TRANS_SANITY_CHECK);

  if (!log.getHasFix() || gps_unreasonable || gps_accuracy_insane || gps_lat_lng_alt_insane || gps_vel_insane) {
    //this->gps_valid = false;
    this->determine_gps_mode(current_time);
    return;
  }

  double sensor_time = current_time - sensor_time_offset;

  // Process message
  //this->gps_valid = true;
  this->gps_mode = true;
  Geodetic geodetic = { log.getLatitude(), log.getLongitude(), log.getAltitude() };
  this->converter = std::make_unique<LocalCoord>(geodetic);

  VectorXd ecef_pos = this->converter->ned2ecef({ 0.0, 0.0, 0.0 }).to_vector();
  VectorXd ecef_vel = this->converter->ned2ecef({ log.getVNED()[0], log.getVNED()[1], log.getVNED()[2] }).to_vector() - ecef_pos;
  float ecef_pos_std = std::sqrt(this->gps_variance_factor * std::pow(log.getHorizontalAccuracy(), 2) + this->gps_vertical_variance_factor * std::pow(log.getVerticalAccuracy(), 2));
  MatrixXdr ecef_pos_R = Vector3d::Constant(std::pow(this->gps_std_factor * ecef_pos_std, 2)).asDiagonal();
  MatrixXdr ecef_vel_R = Vector3d::Constant(std::pow(this->gps_std_factor * log.getSpeedAccuracy(), 2)).asDiagonal();

  this->unix_timestamp_millis = log.getUnixTimestampMillis();
  double gps_est_error = (this->kf->get_x().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START) - ecef_pos).norm();

  VectorXd orientation_ecef = quat2euler(vector2quat(this->kf->get_x().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START)));
  VectorXd orientation_ned = ned_euler_from_ecef({ ecef_pos(0), ecef_pos(1), ecef_pos(2) }, orientation_ecef);
  VectorXd orientation_ned_gps = Vector3d(0.0, 0.0, DEG2RAD(log.getBearingDeg()));
  VectorXd orientation_error = (orientation_ned - orientation_ned_gps).array() - M_PI;
  for (int i = 0; i < orientation_error.size(); i++) {
    orientation_error(i) = std::fmod(orientation_error(i), 2.0 * M_PI);
    if (orientation_error(i) < 0.0) {
      orientation_error(i) += 2.0 * M_PI;
    }
    orientation_error(i) -= M_PI;
  }
  ECEF ecef_pos_struct = { ecef_pos(0), ecef_pos(1), ecef_pos(2) };
  VectorXd ecef_pose_euler = ecef_euler_from_ned(ecef_pos_struct, orientation_ned_gps);
  Quaterniond ecef_pose_quat = euler2quat(ecef_pose_euler);
  VectorXd initial_pose_ecef_quat = quat2vector(ecef_pose_quat);

  if (ecef_vel.norm() > 5.0 && orientation_error.norm() > 1.0) {
    LOGE("Locationd vs ubloxLocation orientation difference too large, kalman reset");
    this->reset_kalman(NAN, initial_pose_ecef_quat, ecef_pos, ecef_vel, ecef_pos_R, ecef_vel_R);
    this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_ORIENTATION_FROM_GPS, { initial_pose_ecef_quat });
  } else if (gps_est_error > 100.0) {
    LOGE("Locationd vs ubloxLocation position difference too large, kalman reset");
    this->reset_kalman(NAN, initial_pose_ecef_quat, ecef_pos, ecef_vel, ecef_pos_R, ecef_vel_R);
  }

  this->last_gps_msg = sensor_time;
  this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_POS, { ecef_pos }, { ecef_pos_R });
  this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_VEL, { ecef_vel }, { ecef_vel_R });
}

void Localizer::handle_gnss(double current_time, const cereal::GnssMeasurements::Reader& log) {

  if (!log.getPositionECEF().getValid() || !log.getVelocityECEF().getValid()) {
    this->determine_gps_mode(current_time);
    return;
  }

  double sensor_time = log.getMeasTime() * 1e-9;
  sensor_time -= this->gps_time_offset;

  auto ecef_pos_v = log.getPositionECEF().getValue();
  VectorXd ecef_pos = Vector3d(ecef_pos_v[0], ecef_pos_v[1], ecef_pos_v[2]);

  // indexed at 0 cause all std values are the same MAE
  auto ecef_pos_std = log.getPositionECEF().getStd()[0];
  MatrixXdr ecef_pos_R = Vector3d::Constant(pow(this->gps_std_factor*ecef_pos_std, 2)).asDiagonal();

  auto ecef_vel_v = log.getVelocityECEF().getValue();
  VectorXd ecef_vel = Vector3d(ecef_vel_v[0], ecef_vel_v[1], ecef_vel_v[2]);

  // indexed at 0 cause all std values are the same MAE
  auto ecef_vel_std = log.getVelocityECEF().getStd()[0];
  MatrixXdr ecef_vel_R = Vector3d::Constant(pow(this->gps_std_factor*ecef_vel_std, 2)).asDiagonal();

  double gps_est_error = (this->kf->get_x().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START) - ecef_pos).norm();

  VectorXd orientation_ecef = quat2euler(vector2quat(this->kf->get_x().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START)));
  VectorXd orientation_ned = ned_euler_from_ecef({ ecef_pos[0], ecef_pos[1], ecef_pos[2] }, orientation_ecef);

  LocalCoord convs((ECEF){ .x = ecef_pos[0], .y = ecef_pos[1], .z = ecef_pos[2] });
  ECEF next_ecef = {.x = ecef_pos[0] + ecef_vel[0], .y = ecef_pos[1] + ecef_vel[1], .z = ecef_pos[2] + ecef_vel[2]};
  VectorXd ned_vel = convs.ecef2ned(next_ecef).to_vector();
  double bearing_rad = atan2(ned_vel[1], ned_vel[0]);

  VectorXd orientation_ned_gps = Vector3d(0.0, 0.0, bearing_rad);
  VectorXd orientation_error = (orientation_ned - orientation_ned_gps).array() - M_PI;
  for (int i = 0; i < orientation_error.size(); i++) {
    orientation_error(i) = std::fmod(orientation_error(i), 2.0 * M_PI);
    if (orientation_error(i) < 0.0) {
      orientation_error(i) += 2.0 * M_PI;
    }
    orientation_error(i) -= M_PI;
  }
  VectorXd initial_pose_ecef_quat = quat2vector(euler2quat(ecef_euler_from_ned({ ecef_pos(0), ecef_pos[1], ecef_pos[2] }, orientation_ned_gps)));

  if (ecef_pos_std > GPS_POS_STD_THRESHOLD || ecef_vel_std > GPS_VEL_STD_THRESHOLD) {
    this->determine_gps_mode(current_time);
    return;
  }

  // prevent jumping gnss measurements (covered lots, standstill...)
  bool orientation_reset = ecef_vel_std < GPS_VEL_STD_RESET_THRESHOLD;
  orientation_reset &= orientation_error.norm() > GPS_ORIENTATION_ERROR_RESET_THRESHOLD;
  orientation_reset &= !this->standstill;
  if (orientation_reset) {
    this->orientation_reset_count++;
  } else {
    this->orientation_reset_count = 0;
  }

  if ((gps_est_error > GPS_POS_ERROR_RESET_THRESHOLD && ecef_pos_std < GPS_POS_STD_RESET_THRESHOLD) || this->last_gps_msg == 0) {
    // always reset on first gps message and if the location is off but the accuracy is high
    LOGE("Locationd vs gnssMeasurement position difference too large, kalman reset");
    this->reset_kalman(NAN, initial_pose_ecef_quat, ecef_pos, ecef_vel, ecef_pos_R, ecef_vel_R);
  } else if (orientation_reset_count > GPS_ORIENTATION_ERROR_RESET_CNT) {
    LOGE("Locationd vs gnssMeasurement orientation difference too large, kalman reset");
    this->reset_kalman(NAN, initial_pose_ecef_quat, ecef_pos, ecef_vel, ecef_pos_R, ecef_vel_R);
    this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_ORIENTATION_FROM_GPS, { initial_pose_ecef_quat });
    this->orientation_reset_count = 0;
  }

  this->gps_mode = true;
  this->last_gps_msg = sensor_time;
  this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_POS, { ecef_pos }, { ecef_pos_R });
  this->kf->predict_and_observe(sensor_time, OBSERVATION_ECEF_VEL, { ecef_vel }, { ecef_vel_R });
}

void Localizer::handle_car_state(double current_time, const cereal::CarState::Reader& log) {
  this->car_speed = std::abs(log.getVEgo());
  this->standstill = log.getStandstill();
  if (this->standstill) {
    this->kf->predict_and_observe(current_time, OBSERVATION_NO_ROT, { Vector3d(0.0, 0.0, 0.0) });
    this->kf->predict_and_observe(current_time, OBSERVATION_NO_ACCEL, { Vector3d(0.0, 0.0, 0.0) });
  }
}

void Localizer::handle_cam_odo(double current_time, const cereal::CameraOdometry::Reader& log) {
  VectorXd rot_device = this->device_from_calib * floatlist2vector(log.getRot());
  VectorXd trans_device = this->device_from_calib * floatlist2vector(log.getTrans());

  if (!this->is_timestamp_valid(current_time)) {
    this->observation_timings_invalid = true;
    return;
  }

  if ((rot_device.norm() > ROTATION_SANITY_CHECK) || (trans_device.norm() > TRANS_SANITY_CHECK)) {
    this->observation_values_invalid["cameraOdometry"] += 1.0;
    return;
  }

  VectorXd rot_calib_std = floatlist2vector(log.getRotStd());
  VectorXd trans_calib_std = floatlist2vector(log.getTransStd());

  if ((rot_calib_std.minCoeff() <= MIN_STD_SANITY_CHECK) || (trans_calib_std.minCoeff() <= MIN_STD_SANITY_CHECK)) {
    this->observation_values_invalid["cameraOdometry"] += 1.0;
    return;
  }

  if ((rot_calib_std.norm() > 10 * ROTATION_SANITY_CHECK) || (trans_calib_std.norm() > 10 * TRANS_SANITY_CHECK)) {
    this->observation_values_invalid["cameraOdometry"] += 1.0;
    return;
  }

  this->posenet_stds.pop_front();
  this->posenet_stds.push_back(trans_calib_std[0]);

  // Multiply by 10 to avoid to high certainty in kalman filter because of temporally correlated noise
  trans_calib_std *= 10.0;
  rot_calib_std *= 10.0;
  MatrixXdr rot_device_cov = rotate_std(this->device_from_calib, rot_calib_std).array().square().matrix().asDiagonal();
  MatrixXdr trans_device_cov = rotate_std(this->device_from_calib, trans_calib_std).array().square().matrix().asDiagonal();
  this->kf->predict_and_observe(current_time, OBSERVATION_CAMERA_ODO_ROTATION,
    { rot_device }, { rot_device_cov });
  this->kf->predict_and_observe(current_time, OBSERVATION_CAMERA_ODO_TRANSLATION,
    { trans_device }, { trans_device_cov });
  this->observation_values_invalid["cameraOdometry"] *= DECAY;
  this->camodo_yawrate_distribution = Vector2d(rot_device[2], rotate_std(this->device_from_calib, rot_calib_std)[2]);
}

void Localizer::handle_live_calib(double current_time, const cereal::LiveCalibrationData::Reader& log) {
  if (!this->is_timestamp_valid(current_time)) {
    this->observation_timings_invalid = true;
    return;
  }

  if (log.getRpyCalib().size() > 0) {
    auto live_calib = floatlist2vector(log.getRpyCalib());
    if ((live_calib.minCoeff() < -CALIB_RPY_SANITY_CHECK) || (live_calib.maxCoeff() > CALIB_RPY_SANITY_CHECK)) {
      this->observation_values_invalid["liveCalibration"] += 1.0;
      return;
    }

    this->calib = live_calib;
    this->device_from_calib = euler2rot(this->calib);
    this->calib_from_device = this->device_from_calib.transpose();
    this->calibrated = log.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED;
    this->observation_values_invalid["liveCalibration"] *= DECAY;
  }
}

void Localizer::reset_kalman(double current_time) {
  const VectorXd &init_x = this->kf->get_initial_x();
  const MatrixXdr &init_P = this->kf->get_initial_P();
  this->reset_kalman(current_time, init_x, init_P);
}

void Localizer::finite_check(double current_time) {
  bool all_finite = this->kf->get_x().array().isFinite().all() or this->kf->get_P().array().isFinite().all();
  if (!all_finite) {
    LOGE("Non-finite values detected, kalman reset");
    this->reset_kalman(current_time);
  }
}

void Localizer::time_check(double current_time) {
  if (std::isnan(this->last_reset_time)) {
    this->last_reset_time = current_time;
  }
  if (std::isnan(this->first_valid_log_time)) {
    this->first_valid_log_time = current_time;
  }
  double filter_time = this->kf->get_filter_time();
  bool big_time_gap = !std::isnan(filter_time) && (current_time - filter_time > 10);
  if (big_time_gap) {
    LOGE("Time gap of over 10s detected, kalman reset");
    this->reset_kalman(current_time);
  }
}

void Localizer::update_reset_tracker() {
  // reset tracker is tuned to trigger when over 1reset/10s over 2min period
  if (this->is_gps_ok()) {
    this->reset_tracker *= RESET_TRACKER_DECAY;
  } else {
    this->reset_tracker = 0.0;
  }
}

void Localizer::reset_kalman(double current_time, const VectorXd &init_orient, const VectorXd &init_pos, const VectorXd &init_vel, const MatrixXdr &init_pos_R, const MatrixXdr &init_vel_R) {
  // too nonlinear to init on completely wrong
  VectorXd current_x = this->kf->get_x();
  MatrixXdr current_P = this->kf->get_P();
  MatrixXdr init_P = this->kf->get_initial_P();
  const MatrixXdr &reset_orientation_P = this->kf->get_reset_orientation_P();
  int non_ecef_state_err_len = init_P.rows() - (STATE_ECEF_POS_ERR_LEN + STATE_ECEF_ORIENTATION_ERR_LEN + STATE_ECEF_VELOCITY_ERR_LEN);

  current_x.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START) = init_orient;
  current_x.segment<STATE_ECEF_VELOCITY_LEN>(STATE_ECEF_VELOCITY_START) = init_vel;
  current_x.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START) = init_pos;

  init_P.block<STATE_ECEF_POS_ERR_LEN, STATE_ECEF_POS_ERR_LEN>(STATE_ECEF_POS_ERR_START, STATE_ECEF_POS_ERR_START).diagonal() = init_pos_R.diagonal();
  init_P.block<STATE_ECEF_ORIENTATION_ERR_LEN, STATE_ECEF_ORIENTATION_ERR_LEN>(STATE_ECEF_ORIENTATION_ERR_START, STATE_ECEF_ORIENTATION_ERR_START).diagonal() = reset_orientation_P.diagonal();
  init_P.block<STATE_ECEF_VELOCITY_ERR_LEN, STATE_ECEF_VELOCITY_ERR_LEN>(STATE_ECEF_VELOCITY_ERR_START, STATE_ECEF_VELOCITY_ERR_START).diagonal() = init_vel_R.diagonal();
  init_P.block(STATE_ANGULAR_VELOCITY_ERR_START, STATE_ANGULAR_VELOCITY_ERR_START, non_ecef_state_err_len, non_ecef_state_err_len).diagonal() = current_P.block(STATE_ANGULAR_VELOCITY_ERR_START,
    STATE_ANGULAR_VELOCITY_ERR_START, non_ecef_state_err_len, non_ecef_state_err_len).diagonal();

  this->reset_kalman(current_time, current_x, init_P);
}

void Localizer::reset_kalman(double current_time, const VectorXd &init_x, const MatrixXdr &init_P) {
  this->kf->init_state(init_x, init_P, current_time);
  this->last_reset_time = current_time;
  this->reset_tracker += 1.0;
}

void Localizer::handle_msg_bytes(const char *data, const size_t size) {
  AlignedBuffer aligned_buf;

  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(data, size));
  cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

  this->handle_msg(event);
}

void Localizer::handle_msg(const cereal::Event::Reader& log) {
  double t = log.getLogMonoTime() * 1e-9;
  this->time_check(t);
  if (log.isAccelerometer()) {
    //if (log.getAccelerometer().getSource() == cereal::SensorEventData::SensorSource::BMX055 && !this->is_jy62()) {
      // 仅在非JY62设备模式下处理BMX055加速度计数据
      this->handle_sensor(t, log.getAccelerometer());
    //}
  } else if (log.isGyroscope()) {
    //if (log.getGyroscope().getSource() == cereal::SensorEventData::SensorSource::BMX055 && !this->is_jy62()) {
      // 仅在非JY62设备模式下处理BMX055陀螺仪数据
      this->handle_sensor(t, log.getGyroscope());
    //}
  } else if (log.isGpsLocation()) {
    this->handle_gps(t, log.getGpsLocation(), GPS_QUECTEL_SENSOR_TIME_OFFSET);
  } else if (log.isGpsLocationExternal()) {
    this->handle_gps(t, log.getGpsLocationExternal(), GPS_UBLOX_SENSOR_TIME_OFFSET);
  //} else if (log.isGnssMeasurements()) {
  //  this->handle_gnss(t, log.getGnssMeasurements());
  } else if (log.isCarState()) {
    this->handle_car_state(t, log.getCarState());
  } else if (log.isCameraOdometry()) {
    this->handle_cam_odo(t, log.getCameraOdometry());
  } else if (log.isLiveCalibration()) {
    this->handle_live_calib(t, log.getLiveCalibration());
  }
  this->finite_check();
  this->update_reset_tracker();
}

kj::ArrayPtr<capnp::byte> Localizer::get_message_bytes(MessageBuilder& msg_builder, bool inputsOK,
                                                       bool sensorsOK, bool gpsOK, bool msgValid) {
  cereal::Event::Builder evt = msg_builder.initEvent();
  evt.setValid(msgValid);
  cereal::LiveLocationKalman::Builder liveLoc = evt.initLiveLocationKalman();
  this->build_live_location(liveLoc);
  liveLoc.setSensorsOK(sensorsOK);
  liveLoc.setGpsOK(gpsOK);
  liveLoc.setInputsOK(inputsOK);
  return msg_builder.toBytes();
}

bool Localizer::is_gps_ok() {
  return (this->kf->get_filter_time() - this->last_gps_msg) < 2.0;
}

bool Localizer::critical_services_valid(const std::map<std::string, double> &critical_services) {
  for (auto &kv : critical_services){
    if (kv.second >= INPUT_INVALID_THRESHOLD){
      return false;
    }
  }
  return true;
}

bool Localizer::is_timestamp_valid(double current_time) {
  double filter_time = this->kf->get_filter_time();
  if (!std::isnan(filter_time) && ((filter_time - current_time) > MAX_FILTER_REWIND_TIME)) {
    LOGE("Observation timestamp is older than the max rewind threshold of the filter");
    return false;
  }
  return true;
}

void Localizer::determine_gps_mode(double current_time) {
  // 1. If the pos_std is greater than what's not acceptable and localizer is in gps-mode, reset to no-gps-mode
  // 2. If the pos_std is greater than what's not acceptable and localizer is in no-gps-mode, fake obs
  // 3. If the pos_std is smaller than what's not acceptable, let gps-mode be whatever it is
  VectorXd current_pos_std = this->kf->get_P().block<STATE_ECEF_POS_ERR_LEN, STATE_ECEF_POS_ERR_LEN>(STATE_ECEF_POS_ERR_START, STATE_ECEF_POS_ERR_START).diagonal().array().sqrt();
  if (current_pos_std.norm() > SANE_GPS_UNCERTAINTY){
    if (this->gps_mode){
      this->gps_mode = false;
      this->reset_kalman(current_time);
    } else {
      this->input_fake_gps_observations(current_time);
    }
  }
}


int Localizer::locationd_thread() {
  Params params;
  // LocalizerGnssSource source;
  const char* gps_location_socket;
  if (params.getBool("UbloxAvailable")) {
    // source = LocalizerGnssSource::UBLOX;
    gps_location_socket = "gpsLocationExternal";
  } else {
    // LocalizerGnssSource source = LocalizerGnssSource::QCOM;
    gps_location_socket = "gpsLocation";
  }

  // Removed configure_gnss_source call since it's not defined
  const std::initializer_list<const char *> service_list = {gps_location_socket, "cameraOdometry", "liveCalibration",
                                                          "carState", "accelerometer", "gyroscope"};

  SubMaster sm(service_list, {}, nullptr, {gps_location_socket});
  PubMaster pm({"liveLocationKalman"});

  // 如果是JY62设备，启动读取线程
  if (this->is_jy62_device_) {
    this->start_jy62_reader();
  }

  uint64_t cnt = 0;
  bool filterInitialized = false;
  const std::vector<std::string> critical_input_services = {"cameraOdometry", "liveCalibration", "accelerometer", "gyroscope"};
  for (std::string service : critical_input_services) {
    this->observation_values_invalid.insert({service, 0.0});
  }

  bool ignore_gps = true;
  while (!do_exit) {
    sm.update();
    if (filterInitialized){
      this->observation_timings_invalid_reset();
      for (const char* service : service_list) {
        if (sm.updated(service) && sm.valid(service)){
          const cereal::Event::Reader log = sm[service];
          this->handle_msg(log);
        }
      }
    } else {
      //filterInitialized = sm.allAliveAndValid();
      bool allValid = true;
      for (const char* service : service_list) {
        if (service != gps_location_socket && !sm.valid(service)) {
          allValid = false;
          break;
        }
      }
      filterInitialized = allValid;
    }

    const char* trigger_msg = "cameraOdometry";
    if (sm.updated(trigger_msg)) {
      bool inputsOK = sm.allValid() && this->are_inputs_ok();
      if (ignore_gps) {
        inputsOK = this->are_inputs_ok();
      }
      bool gpsOK = this->is_gps_ok();
      bool sensorsOK = sm.allAliveAndValid({"accelerometer", "gyroscope"});

      /*
      if (!sm.allValid()) {
        for (const char* service : service_list) {
          if (!sm.valid(service)) {
            printf("Service %s is INVALID! (Alive: %d)\n", service, sm.alive(service));
          }
        }
      }
      printf("InputsOK: %d, SensorsOK: %d, GPSOK: %d, FilterInitialized: %d\n", inputsOK, sensorsOK, gpsOK, filterInitialized);
      */

      // Log time to first fix
      if (gpsOK && std::isnan(this->ttff) && !std::isnan(this->first_valid_log_time)) {
        this->ttff = std::max(1e-3, (sm[trigger_msg].getLogMonoTime() * 1e-9) - this->first_valid_log_time);
      }

      MessageBuilder msg_builder;
      kj::ArrayPtr<capnp::byte> bytes = this->get_message_bytes(msg_builder, inputsOK, sensorsOK, gpsOK, filterInitialized);
      pm.send("liveLocationKalman", bytes.begin(), bytes.size());

      if (cnt % 1200 == 0 && gpsOK) {  // once a minute
        //ignore_gps = false;
        VectorXd posGeo = this->get_position_geodetic();
        std::string lastGPSPosJSON = util::string_format(
          "{\"latitude\": %.15f, \"longitude\": %.15f, \"altitude\": %.15f}", posGeo(0), posGeo(1), posGeo(2));
        params.putNonBlocking("LastGPSPosition", lastGPSPosJSON);
      }
      cnt++;
    }
  }

  // 停止JY62设备读取
  if (this->is_jy62_device_) {
    this->stop_jy62_reader();
  }

  return 0;
}

int main(int argc, char *argv[]) {
  const std::string default_device_path = "/dev/ttyUSB0";
  const int default_baud_rate = 115200;

  std::string device_path = default_device_path;
  int baud_rate = default_baud_rate;
  std::string device_type = "jy62"; // 默认使用JY62设备

  // 解析命令行参数
  int opt;
  const char* short_opts = "d:b:t:";
  const struct option long_opts[] = {
    {"device", required_argument, NULL, 'd'},
    {"baud", required_argument, NULL, 'b'},
    {"type", required_argument, NULL, 't'},
    {NULL, 0, NULL, 0}
  };
  int long_index = 0;

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, &long_index)) != -1) {
    switch (opt) {
      case 'd':
        device_path = optarg;
        break;
      case 'b':
        baud_rate = std::stoi(optarg);
        break;
      case 't':
        device_type = optarg;
        break;
      default:
        fprintf(stderr, "Usage: %s [-d device_path|--device=device_path] [-b baud_rate|--baud=baud_rate] [-t device_type|--type=device_type]\n", argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  LOGW("Starting locationd with device type: %s, device path: %s, baud rate: %d",
       device_type.c_str(), device_path.c_str(), baud_rate);

  Localizer localizer;

  // 如果指定了JY62设备类型，则设置相关参数
  if (device_type == "jy62") {
    localizer.set_device_type(ImuDeviceType::JY62);
    localizer.set_device_params(device_path, baud_rate);
  }

  return localizer.locationd_thread();
}

// 添加JY62设备相关的方法实现
void Localizer::set_device_type(ImuDeviceType type) {
  this->device_type_ = type;
  this->is_jy62_device_ = (type == ImuDeviceType::JY62);
}

bool Localizer::is_jy62() const {
  return (this->device_type_ == ImuDeviceType::JY62);
}

// 设置设备参数
void Localizer::set_device_params(const std::string& device_path, int baud_rate) {
  // 设置设备路径和波特率
  this->device_path_ = device_path;
  this->baud_rate_ = baud_rate;

  LOGW("Device path set to: %s, baud rate set to: %d", this->device_path_.c_str(), this->baud_rate_);
}

// 打开JY62串口设备
int Localizer::open_jy62_device() {
  LOGW("Attempting to open JY62 device at %s with baud rate %d", this->device_path_.c_str(), this->baud_rate_);

  // 打开串口设备
  this->jy62_fd_ = open(this->device_path_.c_str(), O_RDONLY | O_NOCTTY);
  if (this->jy62_fd_ < 0) {
    LOGE("Failed to open JY62 device at %s", this->device_path_.c_str());
    return -1;
  }

  // 配置串口参数
  struct termios tty;
  if (tcgetattr(this->jy62_fd_, &tty) != 0) {
    LOGE("Failed to get terminal attributes for JY62 device");
    close(this->jy62_fd_);
    this->jy62_fd_ = -1;
    return -1;
  }

  // 设置波特率
  speed_t baud_rate;
  switch (this->baud_rate_) {
    case 9600:   baud_rate = B9600;   break;
    case 19200:  baud_rate = B19200;  break;
    case 38400:  baud_rate = B38400;  break;
    case 57600:  baud_rate = B57600;  break;
    case 115200: baud_rate = B115200; break;
    case 230400: baud_rate = B230400; break;
    case 460800: baud_rate = B460800; break;
    case 921600: baud_rate = B921600; break;
    default:
      LOGW("Unsupported baud rate %d, using default 115200", this->baud_rate_);
      baud_rate = B115200;
      break;
  }

  cfsetispeed(&tty, baud_rate);
  cfsetospeed(&tty, baud_rate);

  // 设置数据位为8位
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // 无奇偶校验
  tty.c_cflag &= ~PARENB;

  // 1个停止位
  tty.c_cflag &= ~CSTOPB;

  // 启用接收器
  tty.c_cflag |= CREAD | CLOCAL;

  // 禁用规范模式和回显
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ISIG;

  // 禁用软件流控
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // 禁用输出处理
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  // 设置超时
  tty.c_cc[VMIN] = 0;      // 最小字符数
  tty.c_cc[VTIME] = 10;    // 超时时间(1秒)

  // 应用设置
  if (tcsetattr(this->jy62_fd_, TCSANOW, &tty) != 0) {
    LOGE("Failed to set terminal attributes for JY62 device");
    close(this->jy62_fd_);
    this->jy62_fd_ = -1;
    return -1;
  }

  LOGW("Successfully opened JY62 device at %s with baud rate %d", this->device_path_.c_str(), this->baud_rate_);
  return 0;
}

// 从JY62设备读取一行数据
std::string Localizer::read_jy62_line() {
  if (this->jy62_fd_ < 0) return "";

  char buffer[256];
  ssize_t bytes_read;
  std::string line;

  // 逐字节读取直到换行符或读完缓冲区
  while ((bytes_read = read(this->jy62_fd_, buffer, sizeof(buffer) - 1)) > 0) {
    buffer[bytes_read] = '\0';
    line += buffer;

    // 检查是否包含换行符
    size_t newline_pos = line.find('\n');
    if (newline_pos != std::string::npos) {
      // 截取到换行符的部分作为一行返回
      std::string result = line.substr(0, newline_pos);
      // 保留换行符后的内容供下次读取
      // 这里简化处理，实际应用中可能需要更复杂的缓冲区管理
      return result;
    }
  }

  return line;
}

std::tuple<double, double, double> Localizer::parse_accelerometer(const std::string& line) {
  if (line.find("ACC:") != std::string::npos) {
    std::istringstream iss(line.substr(4));
    double x, y, z;
    if (iss >> x >> y >> z) {
      // 转换为 m/s² (g → m/s²)
      return std::make_tuple(x * 9.8, y * 9.8, z * 9.8);
    }
  }
  return std::make_tuple(0.0, 0.0, 0.0);
}

std::tuple<double, double, double> Localizer::parse_gyroscope(const std::string& line) {
  if (line.find("GYRO:") != std::string::npos) {
    std::istringstream iss(line.substr(5));
    double x, y, z;
    if (iss >> x >> y >> z) {
      // 转换为 rad/s (°/s → rad/s)
      const double DEG_TO_RAD = M_PI / 180.0;
      return std::make_tuple(x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD);
    }
  }
  return std::make_tuple(0.0, 0.0, 0.0);
}

std::tuple<double, double, double> Localizer::parse_angle(const std::string& line) {
  if (line.find("ANGLE:") != std::string::npos) {
    std::istringstream iss(line.substr(6));
    double pitch, roll, yaw;
    if (iss >> pitch >> roll >> yaw) {
      return std::make_tuple(pitch, roll, yaw);
    }
  }
  return std::make_tuple(0.0, 0.0, 0.0);
}

// 发布加速度计数据到系统中
void Localizer::publish_accelerometer(double x, double y, double z) {
  // 创建并发送加速度计消息
  MessageBuilder msg;
  auto event = msg.initEvent();
  uint64_t current_time = nanos_since_boot();
  event.setLogMonoTime(current_time);

  auto accel = event.initAccelerometer();
  accel.setSource(cereal::SensorEventData::SensorSource::BMX055);
  accel.setSensor(SENSOR_ACCELEROMETER);
  accel.setType(SENSOR_TYPE_ACCELEROMETER);
  accel.setTimestamp(current_time);

  auto accelVec = accel.initAcceleration();
  std::vector<float> accelVals = {(float)z, (float)y, (float)(-x)};  // 调整坐标轴顺序以匹配系统约定
  accelVec.setV(kj::ArrayPtr<float>(accelVals.data(), accelVals.size()));

  accel.setVersion(1);

  // 发布消息
  //PubMaster pm({"accelerometer"});
  _pm->send("accelerometer", msg);
}

// 发布陀螺仪数据到系统中
void Localizer::publish_gyroscope(double x, double y, double z) {
  // 创建并发送陀螺仪消息
  MessageBuilder msg;
  auto event = msg.initEvent();
  uint64_t current_time = nanos_since_boot();
  event.setLogMonoTime(current_time);

  auto gyro = event.initGyroscope();
  gyro.setSource(cereal::SensorEventData::SensorSource::BMX055);
  gyro.setSensor(SENSOR_GYRO_UNCALIBRATED);
  gyro.setType(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
  gyro.setTimestamp(current_time);

  auto gyroVec = gyro.initGyroUncalibrated();
  std::vector<float> gyroVals = {(float)z, (float)y, (float)(-x)};  // 调整坐标轴顺序以匹配系统约定
  gyroVec.setV(kj::ArrayPtr<float>(gyroVals.data(), gyroVals.size()));

  gyro.setVersion(1);

  // 发布消息
  //PubMaster pm({"gyroscope"});
  _pm->send("gyroscope", msg);
}

// 发布角度数据到系统中
void Localizer::publish_orientation(double pitch, double roll, double yaw) {
  // 暂时不需要直接发布角度数据，因为系统会通过加速度和陀螺仪数据计算姿态
  LOGD("Orientation data: pitch=%f, roll=%f, yaw=%f", pitch, roll, yaw);
}

// 从JY62设备读取数据包
std::vector<uint8_t> Localizer::read_jy62_packet() {
  if (this->jy62_fd_ < 0) return {};

  static std::vector<uint8_t> buffer;  // 静态缓冲区保存未处理的数据
  uint8_t temp_buffer[256];

  // 先读取新数据到临时缓冲区
  int bytes_read = read(this->jy62_fd_, temp_buffer, sizeof(temp_buffer));

  if (bytes_read > 0) {
    // 将新数据添加到缓冲区
    buffer.insert(buffer.end(), temp_buffer, temp_buffer + bytes_read);
  } else if (bytes_read < 0) {
    // 读取错误
    LOGE("Error reading from JY62 device: %s", strerror(errno));
    return {};
  }

  // 如果缓冲区太大，清理掉旧数据避免内存泄漏
  if (buffer.size() > 1024) {
    buffer.erase(buffer.begin(), buffer.begin() + 512);
  }

  // 尝试从缓冲区中提取完整数据包
  while (buffer.size() >= 11) {
    // 查找数据包起始标记
    size_t start_pos = 0;
    while (start_pos < buffer.size() && buffer[start_pos] != 0x55) {
      start_pos++;
    }

    // 如果没有找到起始标记，清空缓冲区
    if (start_pos >= buffer.size()) {
      buffer.clear();
      break;
    }

    // 如果起始标记不在开头，删除前面的无效数据
    if (start_pos > 0) {
      buffer.erase(buffer.begin(), buffer.begin() + start_pos);
    }

    // 确保有足够的数据构成完整数据包
    if (buffer.size() < 11) {
      break;
    }

    // 检查数据包类型
    uint8_t packet_type = buffer[1];

    // 检查是否为有效的数据包类型
    bool valid_type = (packet_type >= 0x50 && packet_type <= 0x59);
    if (!valid_type) {
      // 未知类型，跳过起始字节并继续查找
      if (!buffer.empty()) {
        buffer.erase(buffer.begin());
      }
      continue;
    }

    // 计算校验和
    uint8_t checksum = 0;
    for (int i = 0; i < 10; ++i) {
      checksum += buffer[i];
    }

    if (checksum != buffer[10]) {
      LOGW("Checksum mismatch: expected 0x%02x, got 0x%02x", checksum, buffer[10]);
      // 校验失败，仅跳过起始字节，而不是整个数据包
      if (!buffer.empty()) {
        buffer.erase(buffer.begin());
      }
      continue;
    }

    // 提取完整且有效的数据包
    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 11);
    buffer.erase(buffer.begin(), buffer.begin() + 11);
    LOGD("Successfully extracted packet of type 0x%02x", packet_type);
    return packet;
  }

  return {};
}

// 根据文档规范解析JY62数据包
bool Localizer::parse_jy62_packet(const std::vector<uint8_t>& packet,
                         double& accel_x, double& accel_y, double& accel_z,
                         double& gyro_x, double& gyro_y, double& gyro_z) {
  // 检查数据包大小
  if (packet.size() < 11) {  // 至少需要11个字节
    cloudlog_e(0, __FILE__, __LINE__, __func__, "Invalid JY62 packet size: %zu", packet.size());
    return false;
  }

  // 检查协议头
  if (packet[0] != 0x55) {
    cloudlog_e(0, __FILE__, __LINE__, __func__, "Invalid JY62 packet header: 0x%02x", packet[0]);
    return false;
  }

  // 计算校验和 (前10个字节之和的低8位)
  uint8_t expected_sum = 0;
  for (int i = 0; i < 10; i++) {
    expected_sum += packet[i];
  }
  uint8_t actual_sum = packet[10];

  if (expected_sum != actual_sum) {
    cloudlog_e(0, __FILE__, __LINE__, __func__, "JY62 packet checksum error: expected 0x%02x, got 0x%02x",
               expected_sum, actual_sum);
    return false;
  }

  // 解析数据包类型
  uint8_t type = packet[1];
  switch (type) {
    case 0x51: { // 加速度数据
      // 检查是否有足够的数据
      if (packet.size() < 11) return false;

      // 解析加速度数据
      int16_t ax = (int16_t)((packet[3] << 8) | packet[2]);
      int16_t ay = (int16_t)((packet[5] << 8) | packet[4]);
      int16_t az = (int16_t)((packet[7] << 8) | packet[6]);

      // 转换为物理单位 (g)->m/s^2
      accel_x = ax / 32768.0 * 16.0 * 9.8;
      accel_y = ay / 32768.0 * 16.0 * 9.8;
      accel_z = az / 32768.0 * 16.0 * 9.8;

      return true;
    }
    case 0x52: { // 角速度数据
      // 检查是否有足够的数据
      if (packet.size() < 11) return false;

      // 解析角速度数据
      int16_t wx = (int16_t)((packet[3] << 8) | packet[2]);
      int16_t wy = (int16_t)((packet[5] << 8) | packet[4]);
      int16_t wz = (int16_t)((packet[7] << 8) | packet[6]);

      // 转换为物理单位 (°/s)->（rad/s）
      gyro_x = wx / 32768.0 * 2000.0 * M_PI / 180.0;
      gyro_y = wy / 32768.0 * 2000.0 * M_PI / 180.0;
      gyro_z = wz / 32768.0 * 2000.0 * M_PI / 180.0;

      return true;
    }
    case 0x53: { // 角度数据
      // 检查是否有足够的数据
      if (packet.size() < 11) return false;

      // 解析角度数据
      int16_t roll_l = (int16_t)((packet[3] << 8) | packet[2]);
      int16_t pitch_l = (int16_t)((packet[5] << 8) | packet[4]);
      int16_t yaw_l = (int16_t)((packet[7] << 8) | packet[6]);

      // 转换为物理单位 (°)
      double roll = roll_l / 32768.0 * 180.0;
      double pitch = pitch_l / 32768.0 * 180.0;
      double yaw = yaw_l / 32768.0 * 180.0;

      // 发布角度数据
      publish_orientation(pitch, roll, yaw);

      return true;
    }
    default: {
      cloudlog_e(0, __FILE__, __LINE__, __func__, "Unknown JY62 packet type: 0x%02x", type);
      return false;
    }
  }
}

// 更新后的JY62设备数据读取线程
void Localizer::jy62_reader_thread() {
  if (!this->is_jy62_device_ || this->jy62_fd_ < 0) return;

  LOGW("Starting JY62 reader thread");

  double accel_x = 0, accel_y = 0, accel_z = 0;
  double gyro_x = 0, gyro_y = 0, gyro_z = 0;

  int read_count = 0;
  bool show_accel = false;
  bool show_gyro = false;

  while (this->jy62_running_) {
    std::vector<uint8_t> packet = this->read_jy62_packet();
    if (!packet.empty()) {
      if(0 == (read_count%20)){
        show_accel = true;
        show_gyro = true;
      }
      read_count++;
      LOGD("Received packet #%d, type: 0x%02x, size: %zu", read_count, packet[1], packet.size());

      // 解析数据包
      if (this->parse_jy62_packet(packet, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)) {
        // 根据数据包类型发布相应的数据
        uint8_t packet_type = packet[1];
        switch (packet_type) {
          case 0x51: // 加速度数据
            this->publish_accelerometer(accel_x, accel_y, accel_z);
            if(show_accel){
              show_accel = false;
              //printf("Accel: x %.3f, y %.3f, z %.3f\n", accel_x, accel_y, accel_z);
            }
            break;
          case 0x52: // 角速度数据
            this->publish_gyroscope(gyro_x, gyro_y, gyro_z);
            if(show_gyro){
              show_gyro = false;
              //printf("Gyro: x %.3f, y %.3f, z %.3f\n", gyro_x, gyro_y, gyro_z);
            }
            break;
          case 0x53: // 角度数据
            // 当前不处理角度数据
            LOGD("Angle packet received and validated");
            break;
          default:
            LOGD("Valid packet of type 0x%02x processed", packet_type);
            break;
        }
      } else {
        LOGD("Failed to parse packet #%d", read_count);
      }
    }

    // 短暂休眠以避免过度占用CPU
    usleep(5000); // 5ms
  }

  LOGW("JY62 reader thread stopped");
}

// 启动JY62设备读取
void Localizer::start_jy62_reader() {
  if (!this->is_jy62_device_) return;

  if (this->open_jy62_device() == 0) {
    this->jy62_running_ = true;
    this->jy62_thread_ = std::thread(&Localizer::jy62_reader_thread, this);
  }
}

// 停止JY62设备读取
void Localizer::stop_jy62_reader() {
  if (this->is_jy62_device_) {
    if (this->jy62_thread_.joinable()) {
      this->jy62_running_ = false;
      this->jy62_thread_.join();
    }

    if (this->jy62_fd_ >= 0) {
      close(this->jy62_fd_);
      this->jy62_fd_ = -1;
    }
  }
}