/**
 * @file setpoint.cpp
 * @brief Class for controlling the input reference angle to the inner loop controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/controller/Setpoint.h"

#include <chrono>
#include <iostream>

#include "flyMS/controller/constants.h"
#include "flyMS/hardware/RemoteController.h"
#include "flyMS/util/debug_mode.h"
#include "spdlog/spdlog.h"

namespace flyMS {

static constexpr float kYAW_DEADZONE_THRESH_RAD_S = 0.05f;  //< in rad/sec

Setpoint::Setpoint(FlightMode flight_mode, std::array<float, 3> max_setpts_stabilized,
                   std::array<float, 3> max_setpts_acro, std::array<float, 2> throttle_limits)
    : yaw_integrator_({0., static_cast<double>(kFLYMS_CONTROL_LOOP_DT)}, {1., -1.}),  // Digital filter integrator
      flight_mode_(flight_mode),
      max_setpoints_stabilized_(max_setpts_stabilized),
      max_setpoints_acro_(max_setpts_acro),
      throttle_limits_(throttle_limits) {
  // Sanity checks
  if (throttle_limits[1] <= throttle_limits[0]) {
    throw std::invalid_argument("Invalid throttle parameters! First param is min, second is max");
  }
  if (flight_mode_ == FlightMode::STABILIZED) {
    if (max_setpoints_stabilized_[0] >= M_PI / 2 || max_setpoints_stabilized_[1] >= M_PI / 2) {
      throw std::invalid_argument("Invalid max_setpoints_stabilized, Roll,Pitch must be < Pi/2 ");
    }
  }
}

Setpoint::Setpoint(FlightMode flight_mode, SetpointConfig config)
    : Setpoint(flight_mode, config.max_setpoints_stabilized, config.max_setpoints_acro, config.throttle_limits) {}

Setpoint::Setpoint(FlightMode flight_mode, const YAML::Node& config_params)
    : Setpoint(flight_mode, from_yaml<SetpointConfig>(config_params)) {}

Setpoint::~Setpoint() {}

std::vector<float> Setpoint::calculate_setpoint_data(const std::vector<float>& remote_control_data) {
  std::vector<float> setpoint_outputs(4);

  // Set the throttle
  remote_control_data[kRC_THROTTLE_INDEX] * (throttle_limits_[1] - throttle_limits_[0]) + throttle_limits_[0];

  // Set roll/pitch reference value
  // DSM2 Receiver is inherently positive to the left
  if (flight_mode_ == FlightMode::STABILIZED) {  // Stabilized Flight Mode
    setpoint_outputs[kRC_ROLL_INDEX] = -remote_control_data[kRC_ROLL_INDEX] * max_setpoints_stabilized_[0];
    setpoint_outputs[kRC_PITCH_INDEX] = remote_control_data[kRC_PITCH_INDEX] * max_setpoints_stabilized_[1];
  } else if (flight_mode_ == FlightMode::ACRO) {
    setpoint_outputs[kRC_ROLL_INDEX] = -remote_control_data[kRC_ROLL_INDEX] * max_setpoints_acro_[0];
    setpoint_outputs[kRC_PITCH_INDEX] = remote_control_data[kRC_PITCH_INDEX] * max_setpoints_acro_[1];
  } else {
    throw std::runtime_error("Error! Invalid flight mode");
  }

  // Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
  // Apply the integration outside of current if statement, needs to run at 200Hz
  auto yaw_rate = remote_control_data[kRC_YAW_INDEX] * max_setpoints_stabilized_[2];

  // Apply a deadzone to keep integrator from wandering
  if (std::abs(yaw_rate) < kYAW_DEADZONE_THRESH_RAD_S) {
    yaw_rate = 0;
  }

  setpoint_outputs[3] = yaw_integrator_.update_filter(yaw_rate);

  return setpoint_outputs;
}

void Setpoint::set_yaw_ref(float ref) { yaw_integrator_.set_to_value(ref); }

}  // namespace flyMS
