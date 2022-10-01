/**
 * @file setpoint.cpp
 * @brief Class for controlling the input reference angle to the inner loop controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/hardware/Setpoint.h"

#include <chrono>
#include <iostream>

#include "flyMS/hardware/RemoteController.h"
#include "flyMS/util/debug_mode.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

static constexpr uint32_t SETPOINT_LOOP_FRQ = 40;
static constexpr uint32_t SETPOINT_LOOP_SLEEP_TIME_US = 1E6 / 40;
static constexpr float DEADZONE_THRESH = 0.05f;

Setpoint::Setpoint(FlightMode flight_mode, std::array<float, 3> max_setpts_stabilized,
                   std::array<float, 3> max_setpts_acro, std::array<float, 2> throttle_limits)
    : setpoint_mutex_(std::make_unique<std::mutex>()),
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

Setpoint::Setpoint(SetpointConfig config)
    : Setpoint(static_cast<FlightMode>(config.flight_mode), config.max_setpoints_stabilized, config.max_setpoints_acro,
               config.throttle_limits) {}

Setpoint::Setpoint(const YAML::Node& config_params) : Setpoint(from_yaml<SetpointConfig>(config_params)) {}

Setpoint::~Setpoint() {}

void Setpoint::wait_for_data_packet() {
  auto& remote = RemoteController::get_instance();
  while (!remote.has_received_data() && rc_get_state() != EXITING) {
    std::this_thread::sleep_for(std::chrono::microseconds(SETPOINT_LOOP_SLEEP_TIME_US));
  }
}

SetpointData Setpoint::get_setpoint_data() {
  auto& remote = RemoteController::get_instance();
  auto rc_channel_data = remote.get_channel_values();

  std::scoped_lock<std::mutex> lock(*setpoint_mutex_);
  // Set roll/pitch reference value
  // DSM2 Receiver is inherently positive to the left
  if (flight_mode_ == FlightMode::STABILIZED) {  // Stabilized Flight Mode
    setpoint_data_.euler_ref[0] = -rc_channel_data[kRC_ROLL_INDEX] * max_setpoints_stabilized_[0];
    setpoint_data_.euler_ref[1] = rc_channel_data[kRC_PITCH_INDEX] * max_setpoints_stabilized_[1];
  } else if (flight_mode_ == FlightMode::ACRO) {
    setpoint_data_.euler_ref[0] = -rc_channel_data[kRC_ROLL_INDEX] * max_setpoints_acro_[0];
    setpoint_data_.euler_ref[1] = rc_channel_data[kRC_PITCH_INDEX] * max_setpoints_acro_[1];
  } else {
    throw std::runtime_error("Error! Invalid flight mode");
  }

  // Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
  // Apply the integration outside of current if statement, needs to run at 200Hz
  setpoint_data_.yaw_rate_ref[1] = setpoint_data_.yaw_rate_ref[0];
  setpoint_data_.yaw_rate_ref[0] = rc_channel_data[kRC_YAW_INDEX] * max_setpoints_stabilized_[2];

  // Apply a deadzone to keep integrator from wandering
  if (std::abs(setpoint_data_.yaw_rate_ref[0]) < DEADZONE_THRESH) {
    setpoint_data_.yaw_rate_ref[0] = 0;
  }

  // Kill Switch
  setpoint_data_.kill_switch = rc_channel_data[kRC_KILL_SWITCH_INDEX];

  // Auxillary Switch
  setpoint_data_.Aux[1] = setpoint_data_.Aux[0];
  setpoint_data_.Aux[0] = rc_channel_data[kRC_FLIGHT_MODE_INDEX];

  // Set the throttle
  setpoint_data_.throttle =
      (rc_channel_data[kRC_THROTTLE_INDEX] + 1.0) / 2.0 * (throttle_limits_[1] - throttle_limits_[0]) +
      throttle_limits_[0];

  // Finally Update the integrator on the yaw reference value
  setpoint_data_.euler_ref[2] =
      setpoint_data_.euler_ref[2] +
      (setpoint_data_.yaw_rate_ref[0] + setpoint_data_.yaw_rate_ref[1]) * 1 / (2 * SETPOINT_LOOP_FRQ);

  return setpoint_data_;
}

void Setpoint::set_yaw_ref(float ref) { setpoint_data_.euler_ref[2] = ref; }

}  // namespace flyMS
