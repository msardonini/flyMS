/**
 * @file state_data.h
 * @author Mike Sardonini
 * @brief Declaration of StateData struct
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "Eigen/Dense"

namespace flyMS {

struct StateData {
  uint64_t timestamp_us;

  Eigen::Vector3f euler;      // Euler angles of aircraft (in roll, pitch, yaw)
  Eigen::Vector3f eulerRate;  // First derivative of euler angles (in roll/s, pitch/s, yaw/s)

  Eigen::Vector3f accel;  // Raw data from accelerometer
  Eigen::Vector3f gyro;   // Raw data from gyroscope
  Eigen::Vector3f mag;    // Raw data from magnetometer

  float barometerAltitude;
  float compassHeading;

  // int    num_wraps;        // Number of spins in Yaw
  float initialYaw;

  /// @brief Instantiates an objects and sets all fields to zero
  static StateData zeros() {
    auto zeros_vec = Eigen::Vector3f::Zero();

    return StateData{.timestamp_us = 0,
                     .euler = zeros_vec,
                     .eulerRate = zeros_vec,
                     .accel = zeros_vec,
                     .gyro = zeros_vec,
                     .mag = zeros_vec,
                     .barometerAltitude = 0.f,
                     .compassHeading = 0.f,
                     .initialYaw = 0.f};
  }
};

}  // namespace flyMS
