/**
 * @file setpoint.hpp
 * @brief Class for controlling the input reference angle to the inner loop controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "flyMS/position_controller.h"
#include "flyMS/types/flight_mode.h"
#include "rc/dsm.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

struct SetpointData {
  std::array<float, 3> euler_ref;     //< Reference (Desired) Position in Roll, Pitch, Yaw
  std::array<float, 2> yaw_rate_ref;  //< Reference Yaw Rate, current (ind 0) and previous (ind 1)
  std::array<float, 2> Aux;           //< The Aux channel value, current (ind 0) and previous
  float throttle;                     //< Throttle given to the drone
  float kill_switch;                  //< Value of the kill switch channel
};

class Setpoint {
 public:
  Setpoint(const YAML::Node& config_params);

  // Default Destructor
  ~Setpoint();

  // Forbid c'tors we do not want
  Setpoint() = delete;
  Setpoint(const Setpoint&) = delete;
  Setpoint& operator=(const Setpoint&) = delete;

  /**
   * @brief Initialize the object. Start the internal thread and initialize communication with the remote controller
   *
   * @return int 0 on success, -1 on failure
   */
  void init();

  /**
   * @brief Blocks execution until a data packet is received. Also exits if rc_get_state() == EXITING
   *
   */
  void wait_for_data_packet();

  /**
   * @brief      Gets the setpoint data.
   *
   * @return     The setpoint data.
   */
  SetpointData GetSetpointData();

  /**
   * @brief Set the Yaw Setpoint value to a user defined offset
   *
   * @param ref the reference value to set
   */
  void SetYawRef(float ref);

  /**
   * @brief Get the Min Throttle
   *
   * @return float
   */
  float GetMinThrottle() const { return throttle_limits_[0]; }

 private:
  int SetpointManager();

  std::atomic<bool> is_running_;         //< Flag to indicate if the object is running vs shutting down
  std::atomic<bool> has_received_data_;  //< True if any dsm packet has been received

  std::thread setpoint_thread_;  //< Thread that reads dsm2 UART
  std::mutex setpoint_mutex_;    //< Mutex that protects setpoint_data_
  SetpointData setpoint_data_;   //< Data to be shared with the user when called

  // All configurable parameters
  bool is_debug_mode_;                             //< is running in debug mode
  std::array<float, 3> max_setpoints_stabilized_;  //< Max orientation commands in [rad, rad, rad/s]
  std::array<float, 3> max_setpoints_acro_;        //< Max orientation commands in acro mode [rad/s, rad/s, rad/s]
  std::array<float, 2> throttle_limits_;           //< Min and max throttle limits
  FlightMode flight_mode_;                         //< The flight mode
  // bool is_headless_mode_;  //< Not currently supported
};

}  // namespace flyMS
