#pragma once

#include <algorithm>
#include <array>

#include "flyMS/controller/DigitalFilter.hpp"
#include "flyMS/controller/constants.h"
#include "flyMS/hardware/RemoteController.h"
#include "flyMS/types/flight_mode.h"
#include "flyMS/types/state_data.h"
#include "flyMS/util/yaml_serialization.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

struct AttitudeControllerConfig {
  std::array<double, 3> roll_PID_inner;
  std::array<double, 3> roll_PID_outer;
  std::array<double, 3> pitch_PID_inner;
  std::array<double, 3> pitch_PID_outer;
  std::array<double, 3> yaw_PID;
  std::array<float, 3> max_control_effort;
  double pid_LPF_const_sec;

  constexpr static auto properties =
      std::make_tuple(property(&AttitudeControllerConfig::roll_PID_inner, "roll_PID_inner"),
                      property(&AttitudeControllerConfig::roll_PID_outer, "roll_PID_outer"),
                      property(&AttitudeControllerConfig::pitch_PID_inner, "pitch_PID_inner"),
                      property(&AttitudeControllerConfig::pitch_PID_outer, "pitch_PID_outer"),
                      property(&AttitudeControllerConfig::yaw_PID, "yaw_PID"),
                      property(&AttitudeControllerConfig::max_control_effort, "max_control_effort"),
                      property(&AttitudeControllerConfig::pid_LPF_const_sec, "pid_LPF_const_sec")

      );
};

class AttitudeController {
 public:
  explicit AttitudeController(const AttitudeControllerConfig& config)
      : roll_inner_pid_(generate_pid(config.roll_PID_inner, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
        roll_outer_pid_(generate_pid(config.roll_PID_outer, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
        pitch_inner_pid_(generate_pid(config.pitch_PID_inner, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
        pitch_outer_pid_(generate_pid(config.pitch_PID_outer, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
        yaw_pid_(generate_pid(config.yaw_PID, config.pid_LPF_const_sec, kFLYMS_CONTROL_LOOP_DT)),
        max_control_effort_(config.max_control_effort) {
    spdlog::debug("roll_PID_inner pid P {}, I {}, D {}", config.roll_PID_inner[0], config.roll_PID_inner[1],
                  config.roll_PID_inner[2]);
  }

  explicit AttitudeController(const YAML::Node& controller_params)
      : AttitudeController(from_yaml<AttitudeControllerConfig>(controller_params)) {}

  std::vector<float> calculate_control_loop(const std::vector<float>& setpoints, const StateData& imu_data_body,
                                            FlightMode flight_mode) {
    std::vector<float> state_control_efforts(4);

    float roll_rate_setpoint;
    float pitch_rate_setpoint;
    if (flight_mode == FlightMode::STABILIZED) {
      roll_rate_setpoint = roll_outer_pid_.update_filter(setpoints[kRC_ROLL_INDEX] - imu_data_body.euler[0]);
      pitch_rate_setpoint = pitch_outer_pid_.update_filter(setpoints[kRC_PITCH_INDEX] - imu_data_body.euler[1]);
    } else if (flight_mode == FlightMode::ACRO) {
      roll_rate_setpoint = setpoints[kRC_ROLL_INDEX];
      pitch_rate_setpoint = setpoints[kRC_PITCH_INDEX];
    } else {
      throw std::runtime_error("[FlightCore] Error! Invalid flight mode. Shutting down now");
    }

    // Roll Inner Loop
    state_control_efforts[kRC_ROLL_INDEX] =
        roll_inner_pid_.update_filter(roll_rate_setpoint - imu_data_body.eulerRate[0]);
    state_control_efforts[kRC_ROLL_INDEX] =
        std::clamp(state_control_efforts[kRC_ROLL_INDEX], -max_control_effort_[0], max_control_effort_[0]);

    // Pitch Inner Loop
    state_control_efforts[kRC_PITCH_INDEX] =
        pitch_inner_pid_.update_filter(pitch_rate_setpoint - imu_data_body.eulerRate[1]);
    state_control_efforts[kRC_PITCH_INDEX] =
        std::clamp(state_control_efforts[kRC_PITCH_INDEX], -max_control_effort_[1], max_control_effort_[1]);

    // Yaw Controller
    state_control_efforts[kRC_YAW_INDEX] = yaw_pid_.update_filter(setpoints[kRC_YAW_INDEX] - imu_data_body.euler[2]);
    // Apply a saturation filter
    state_control_efforts[kRC_YAW_INDEX] =
        std::clamp(state_control_efforts[kRC_YAW_INDEX], -max_control_effort_[2], max_control_effort_[2]);

    // Throttle Controller
    state_control_efforts[kRC_THROTTLE_INDEX] = setpoints[kRC_THROTTLE_INDEX];

    return state_control_efforts;
  }

  void zero_pids() {
    roll_inner_pid_.zero_values();
    roll_outer_pid_.zero_values();
    pitch_inner_pid_.zero_values();
    pitch_outer_pid_.zero_values();
    yaw_pid_.zero_values();
  }

 private:
  DigitalFilter roll_inner_pid_;
  DigitalFilter roll_outer_pid_;
  DigitalFilter pitch_inner_pid_;
  DigitalFilter pitch_outer_pid_;
  DigitalFilter yaw_pid_;
  std::array<float, 3> max_control_effort_;  //< Maximum values allowed to to exert in each euler angle DoF
};

}  // namespace flyMS
