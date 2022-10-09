/**
 * @file Imu.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Declaration of Imu class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <functional>

#include "Eigen/Dense"
#include "flyMS/types/state_data.h"
#include "rc/led.h"
#include "rc/mpu.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

/**
 * @brief Singleton class to communicate with the IMU. We use singleton because there is only one imu in hardware. This
 * class utilizes  the DMP (digital motion processor) on the MPU-9250 as a way to calculate orientation
 *
 */
class Imu {
 public:
  // TODO: Test without a callback function, log time, and see if gaps still occur
  // Prevent copies of the singleton Imu class
  Imu(Imu const &) = delete;
  void operator=(Imu const &) = delete;

  /**
   * @brief Get the Instance of the Imu object
   *
   * @return Imu&
   */
  static Imu &getInstance() {
    static Imu instance;  // Guaranteed to be destroyed.
                          // Instantiated on first use.
    return instance;
  }

  /**
   * @brief Initialize the Imu class
   *
   * @param config YAML configuration for the Imu class
   * @param loop_frq Desired sampling rate in Hz of the IMU
   * @param func callback function to use when data is received
   * @return 0 on sucess, -1 on failure
   */
  int init(const YAML::Node &config, uint32_t loop_frq, std::function<void(StateData &)> &func);

 private:
  Imu() = default;

  /**
   * @brief Callback function whenever IMU data is ready
   *
   */
  void dmp_callback();

  /**
   * @brief Callback used with the robotics_cape C api, needs to be static
   *
   */
  static void static_dmp_callback();

  /**
   * @brief Detects a wrap in the yaw value and unwraps it. Required for steady control
   *
   * @param yaw The yaw value to check and potentially update
   */
  void UnwrapYaw(float &yaw);

  int num_wraps_ = 0;                      //< Number of wraps we have turned in yaw
  float yaw_prev_ = 0.f;                   //< The yaw value from the previous iteration
  Eigen::Matrix<float, 3, 3> R_imu_body_;  //< Rotation matrix for imu to body conversions
  Eigen::Matrix<float, 3, 3>
      R_imu_body_dmp_;  //< 3D rotation that transforms the DMP coordinate system to the drone's body frame, which is
                        // usually different than the coordinate system of the IMU
  rc_mpu_data_t imu_data_;  //< data struct that is used to pass imu values from the API
  std::function<void(StateData &)>
      user_func_;  //< The callback fuction provided by the user to call with populated imu data
};

}  // namespace flyMS
