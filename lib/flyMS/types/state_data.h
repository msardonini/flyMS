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
};

}  // namespace flyMS
