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
  explicit AttitudeController(const AttitudeControllerConfig& config);

  explicit AttitudeController(const YAML::Node& controller_params);

  std::vector<float> calculate_control_loop(const std::vector<float>& setpoints, const StateData& imu_data_body,
                                            FlightMode flight_mode);

  void zero_pids();

 private:
  DigitalFilter roll_inner_pid_;
  DigitalFilter roll_outer_pid_;
  DigitalFilter pitch_inner_pid_;
  DigitalFilter pitch_outer_pid_;
  DigitalFilter yaw_pid_;
  std::array<float, 3> max_control_effort_;  //< Maximum values allowed to to exert in each euler angle DoF
};

}  // namespace flyMS
