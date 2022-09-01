#include "flyMS/position_controller.h"

#include <algorithm>
#include <iostream>

#include "flyMS/imu/Imu.h"

namespace flyMS {

PositionController::PositionController(std::array<std::array<double, 3>, 2> pid_coeffs_x,
                                       std::array<std::array<double, 3>, 2> pid_coeffs_y,
                                       std::array<std::array<double, 3>, 2> pid_coeffs_z,
                                       std::array<float, 3> RPY_saturation_limits) {
  for (int i = 0; i < 2; i++) {
    pid_[0][i] = generate_pid(pid_coeffs_x[i], 0.15, LOOP_DELTA_T);
    pid_[0][i] = generate_pid(pid_coeffs_y[i], 0.15, LOOP_DELTA_T);
    pid_[0][i] = generate_pid(pid_coeffs_z[i], 0.15, LOOP_DELTA_T);
  }

  // Conversion matrix to map roll, pitch, throttle commands from PID output in XYZ frame
  XYZ_to_RollPitchThrottle_ << 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

  // Zero out the reference values
  setpoint_position_ = Eigen::Vector3f::Zero();
  setpoint_velocity_ = Eigen::Vector3f::Zero();
  setpoint_orientation_ = Eigen::Vector3f::Zero();
}

PositionController::PositionController(const YAML::Node &config_params)
    : PositionController({config_params["pid_coeffs_x_outer"].as<std::array<double, 3>>(),
                          config_params["pid_coeffs_x_inner"].as<std::array<double, 3>>()},
                         {config_params["pid_coeffs_y_outer"].as<std::array<double, 3>>(),
                          config_params["pid_coeffs_y_inner"].as<std::array<double, 3>>()},
                         {config_params["pid_coeffs_z_outer"].as<std::array<double, 3>>(),
                          config_params["pid_coeffs_z_inner"].as<std::array<double, 3>>()},
                         config_params["RPY_saturation_limits"].as<std::array<float, 3>>()) {}

PositionController::~PositionController() {}

void PositionController::ReceiveVio(const VioData &vio) {
  Eigen::Vector3f setpoint_orientation_xyz;
  // Calculate the PIDs for the outer and inner loops on XYZ axis
  for (int i = 0; i < 3; i++) {
    setpoint_velocity_[i] = pid_[i][0].update_filter(setpoint_position_[i] - vio.position[i]);
    setpoint_orientation_xyz(i) = pid_[i][1].update_filter(setpoint_velocity_(i) - vio.velocity(i));
  }

  // lock the mutex to protect our output variable
  std::lock_guard<std::mutex> lock(*output_mutex_);

  // Account for the change in orientation of the aircraft during its flight
  setpoint_orientation_xyz = vio.quat.inverse() * setpoint_orientation_xyz;

  // Convert to the Roll Pitch Yaw Coordinate System
  setpoint_orientation_ = XYZ_to_RollPitchThrottle_ * setpoint_orientation_xyz;

  // Apply a saturation filter to keep the behavior in check
  for (int i = 0; i < 3; i++) {
    setpoint_orientation_(i) =
        std::clamp(setpoint_orientation_(i), -RPY_saturation_limits_[0], RPY_saturation_limits_[0]);
  }

  // Back out the yaw euler angle which is needed for reference
  Eigen::Matrix3f rotation_mat = vio.quat.toRotationMatrix();
  const float pitch = -asin(rotation_mat(2, 0));
  yaw_ = atan2(rotation_mat(1, 0) / cos(pitch), rotation_mat(0, 0) / cos(pitch));
}

std::tuple<Eigen::Vector3f, float> PositionController::GetSetpoint() const {
  std::lock_guard<std::mutex> lock(*output_mutex_);
  return std::make_tuple(setpoint_orientation_, yaw_);
}

void PositionController::SetReferencePosition(const Eigen::Vector3f &position) {
  std::lock_guard<std::mutex> lock(*output_mutex_);
  setpoint_position_ = position;
}

void PositionController::ResetController() {
  std::lock_guard<std::mutex> lock(*output_mutex_);
  setpoint_position_ = Eigen::Vector3f::Zero();
  setpoint_velocity_ = Eigen::Vector3f::Zero();
  setpoint_orientation_ = Eigen::Vector3f::Zero();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      pid_[i][j].zero_values();
    }
  }
}

}  // namespace flyMS
