#pragma once

#include <mutex>

#include "Eigen/Dense"
#include "flyMS/DigitalFilter.hpp"
#include "flyMS/types/vio_data.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

class PositionController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionController(const YAML::Node &config_params);
  ~PositionController();

  int ReceiveVio(const VioData &vio);

  int GetSetpoint(Eigen::Vector3f &setpoint_orientation, float &yaw);

  int SetReferencePosition(const Eigen::Vector3f &position);

  void ResetController();

 private:
  // Setpoint Position, velocity and orientation
  Eigen::Vector3f setpoint_position_;
  Eigen::Vector3f setpoint_velocity_;
  Eigen::Vector3f setpoint_orientation_;
  float yaw_;

  // Roll pitch and yaw saturdation limits, do not exceed these values
  std::array<float, 3> RPY_saturation_limits_;

  // Conversion matrix to map roll, pitch, throttle commands from PID output in XYZ frame
  Eigen::Matrix3f XYZ_to_RollPitchThrottle_;

  // Filters to calculate the PID output. First dimention is X,Y,Z controllers, second is outer
  // loop and inner loop. Ex. pid_[0][1] is inner loop for X axis, pid_[3][0] is outer loop for
  // Z axis
  std::array<std::array<DigitalFilter, 2>, 3> pid_;

  std::mutex output_mutex_;
};

}  // namespace flyMS
