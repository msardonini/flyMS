/**
 * @file Imu.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Definition of Imu class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "flyMS/hardware/Imu.h"

#include <iostream>

#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr float D2Rf = M_PI / 180.f;

// Get the instance of the singleton class
static Imu &imu_instance = Imu::getInstance();

void Imu::static_dmp_callback() { imu_instance.dmp_callback(); }

void Imu::dmp_callback() {
  if (rc_get_state() == EXITING) {
    return;
  }

  StateData imu_data_imu;   //< State data from the IMU Frame
  StateData imu_data_body;  //< State data from the Body Frame
  for (int i = 0; i < 3; i++) {
    imu_data_imu.euler(i) = imu_data_.dmp_TaitBryan[i];
    imu_data_imu.eulerRate(i) = imu_data_.gyro[i] * D2Rf;
    imu_data_imu.mag(i) = imu_data_.mag[i];
    imu_data_imu.gyro(i) = imu_data_.gyro[i];
    imu_data_imu.accel(i) = imu_data_.accel[i];
  }

  // Convert from IMU frame to body Frame
  imu_data_body.mag = R_imu_body_ * imu_data_imu.mag;
  imu_data_body.gyro = R_imu_body_ * imu_data_imu.gyro;
  imu_data_body.accel = R_imu_body_ * imu_data_imu.accel;
  imu_data_body.euler = R_imu_body_dmp_ * imu_data_imu.euler;
  imu_data_body.eulerRate = R_imu_body_dmp_ * imu_data_imu.eulerRate;

  UnwrapYaw(imu_data_body.euler(2));

  imu_data_body.timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count() -
      rc_mpu_nanos_since_last_dmp_interrupt() / 1000;
  user_func_(imu_data_body);
}

int Imu::init(const YAML::Node &config, std::function<void(StateData &)> &user_func) {
  // Load the transform from imu to body frame
  R_imu_body_ = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(config["R_imu_body"].as<std::array<float, 9> >().data());
  R_imu_body_dmp_ = R_imu_body_;

  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = 2;
  conf.gpio_interrupt_pin_chip = 3;
  conf.gpio_interrupt_pin = 21;
  conf.enable_magnetometer = 1;
  conf.dmp_interrupt_sched_policy = SCHED_FIFO;
  conf.dmp_interrupt_priority = 99;
  conf.dmp_fetch_accel_gyro = 1;
  conf.dmp_sample_rate = LOOP_FREQUENCY;
  conf.orient = ORIENTATION_Z_UP;

  // Check our DCM for the proper orientation config parameter
  constexpr float thresh = 0.95f;
  if (R_imu_body_(0, 2) > thresh) {
    conf.orient = ORIENTATION_X_UP;
    throw std::invalid_argument("X up orientation not yet tested\n");
  } else if (R_imu_body_(0, 2) < -thresh) {
    conf.orient = ORIENTATION_X_DOWN;
    throw std::invalid_argument("X down orientation not yet tested\n");
  } else if (R_imu_body_(1, 2) > thresh) {
    conf.orient = ORIENTATION_Y_UP;
    throw std::invalid_argument("Y up orientation not yet tested\n");
  } else if (R_imu_body_(1, 2) < -thresh) {
    conf.orient = ORIENTATION_Y_DOWN;
    throw std::invalid_argument("Y down orientation not yet tested\n");
  } else if (R_imu_body_(2, 2) > thresh) {
    conf.orient = ORIENTATION_Z_UP;
  } else if (R_imu_body_(2, 2) < -thresh) {
    conf.orient = ORIENTATION_Z_DOWN;
    Eigen::Matrix<float, 3, 3> dmp_conv;
    dmp_conv << 1.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, -1.f;
    R_imu_body_dmp_ = dmp_conv * R_imu_body_dmp_;
  } else {
    std::string err_msg(
        "Error! In order to be in DMP mode, "
        "one of the X,Y,Z vectors on the IMU needs to be parallel with Gravity\n");
    spdlog::error(err_msg);
    rc_led_set(RC_LED_RED, 1);
    rc_led_set(RC_LED_GREEN, 0);
    throw std::invalid_argument(err_msg);
  }

  if (rc_mpu_initialize_dmp(&imu_data_, conf)) {
    spdlog::error("rc_mpu_initialize failed");
    throw std::invalid_argument("rc_mpu_initialize failed");
  }

  user_func_ = user_func;
  rc_mpu_set_dmp_callback(&static_dmp_callback);
  return 0;
}

void Imu::UnwrapYaw(float &yaw) {
  yaw += num_wraps_ * 2 * M_PI;
  if (fabs(yaw - yaw_prev_) > 5) {
    if (yaw > yaw_prev_) {
      num_wraps_--;
      yaw -= 2 * M_PI;
    } else {
      num_wraps_++;
      yaw += 2 * M_PI;
    }
  }
  yaw_prev_ = yaw;
}

}  // namespace flyMS
