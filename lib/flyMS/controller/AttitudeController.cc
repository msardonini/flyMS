#include "flyMS/controller/AttitudeController.h"

#include "spdlog/spdlog.h"

namespace flyMS {

AttitudeController::AttitudeController(const AttitudeControllerConfig& config)
    : roll_inner_pid_(generate_pid(config.roll_PID_inner, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
      roll_outer_pid_(generate_pid(config.roll_PID_outer, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
      pitch_inner_pid_(generate_pid(config.pitch_PID_inner, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
      pitch_outer_pid_(generate_pid(config.pitch_PID_outer, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
      yaw_pid_(generate_pid(config.yaw_PID, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
      max_control_effort_(config.max_control_effort) {
  spdlog::debug("roll_PID_inner pid P {}, I {}, D {}", config.roll_PID_inner[0], config.roll_PID_inner[1],
                config.roll_PID_inner[2]);
}

AttitudeController::AttitudeController(const YAML::Node& controller_params)
    : AttitudeController(from_yaml<AttitudeControllerConfig>(controller_params)) {}

TRPY<float> AttitudeController::calculate_control_loop(const TRPY<float>& setpoints, const StateData& imu_data_body,
                                                       FlightMode flight_mode) {
  TRPY<float> control_efforts;

  float roll_rate_setpoint;
  float pitch_rate_setpoint;
  if (flight_mode == FlightMode::STABILIZED) {
    roll_rate_setpoint = roll_outer_pid_.update_filter(setpoints.roll() - imu_data_body.euler[0]);
    pitch_rate_setpoint = pitch_outer_pid_.update_filter(setpoints.pitch() - imu_data_body.euler[1]);
  } else if (flight_mode == FlightMode::ACRO) {
    roll_rate_setpoint = setpoints.roll();
    pitch_rate_setpoint = setpoints.pitch();
  } else {
    throw std::runtime_error("[FlightCore] Error! Invalid flight mode. Shutting down now");
  }

  // Roll Inner Loop
  control_efforts.roll() = roll_inner_pid_.update_filter(roll_rate_setpoint - imu_data_body.eulerRate[0]);
  control_efforts.roll() = std::clamp(control_efforts.roll(), -max_control_effort_[0], max_control_effort_[0]);

  // Pitch Inner Loop
  control_efforts.pitch() = pitch_inner_pid_.update_filter(pitch_rate_setpoint - imu_data_body.eulerRate[1]);
  control_efforts.pitch() = std::clamp(control_efforts.pitch(), -max_control_effort_[1], max_control_effort_[1]);

  // Yaw Controller
  control_efforts.yaw() = yaw_pid_.update_filter(setpoints.yaw() - imu_data_body.euler[2]);
  control_efforts.yaw() = std::clamp(control_efforts.yaw(), -max_control_effort_[2], max_control_effort_[2]);

  // Throttle Controller
  control_efforts.throttle() = setpoints.throttle();

  return control_efforts;
}

void AttitudeController::zero_pids() {
  roll_inner_pid_.zero_values();
  roll_outer_pid_.zero_values();
  pitch_inner_pid_.zero_values();
  pitch_outer_pid_.zero_values();
  yaw_pid_.zero_values();
}

}  // namespace flyMS
