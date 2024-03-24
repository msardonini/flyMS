#pragma once

#include <algorithm>
#include <array>

#include "flyMS/common/constants.h"
#include "flyMS/common/flight_mode.h"
#include "flyMS/common/state_data.h"
#include "flyMS/common/trpy.h"
#include "flyMS/controller/DigitalFilter.h"
#include "flyMS/hardware/RemoteController.h"
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

/**
 * @brief Execute the PID controllers for the system, and convert the system's measured state and desired state to
 * control commands to be executed by the motors.
 *
 */
class AttitudeController {
 public:
  /**
   * @brief Construct a new Attitude Controller object.
   *
   * @param config The configuration parameters for the AttitudeController
   */
  explicit AttitudeController(const AttitudeControllerConfig& config);

  /**
   * @brief Construct a new Attitude Controller object with a YAML::Node
   *
   * @param controller_params The configuration parameters for the AttitudeController in YAML::Node format
   */
  explicit AttitudeController(const YAML::Node& controller_params);

  /**
   * @brief Calculate the control loop for one time step. The input is the current measured state of the system, and the
   * desired state of the system, the output is the control commands. Depending on the flight mode of the system, the
   * 'setpoints` can mean different things. In stabilized flight mode, the `setpoints` for roll and pitch are the
   * desired angle in radians. In acro mode, the `setpoints` for roll and pitch are the desired angular velocity in
   * radians/s. Yaw is always controlled as angular velocity (radians/s)
   *
   * @param setpoints The desired system state
   * @param imu_data_body The measured system state
   * @param flight_mode The current flight mode of the system
   * @return TRPW<float> The calculated control input
   */
  TRPY<float> calculate_control_loop(const TRPY<float>& setpoints, const StateData& imu_data_body,
                                     FlightMode flight_mode);

  /**
   * @brief Zeros the signals for all internal PID filters
   *
   */
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
