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

/**
 * @brief Data struct that holds the flight commands for the inner loop controller
 *
 */
struct SetpointData {
  std::array<float, 3> euler_ref;     //< Reference (Desired) Position in Roll, Pitch, Yaw
  std::array<float, 2> yaw_rate_ref;  //< Reference Yaw Rate, current (ind 0) and previous (ind 1)
  std::array<float, 2> Aux;           //< The Aux channel value, current (ind 0) and previous
  float throttle;                     //< Throttle given to the drone
  float kill_switch;                  //< Value of the kill switch channel
};

/**
 * @brief Setpoint class. Communicates with the remote control (using robotics_cape rc_dsm interface) to receive
 * commands. Commands are: throttle, roll, pitch, yaw, kill_switch, and Aux_switch. Switches have binary information
 *
 */
class Setpoint {
 public:
  // TODO move is_debug_mode to a compile time constant
  /**
   * @brief Construct a new Setpoint object
   *
   * @param flight_mode  flight mode: stabilized or acro
   * @param max_setpts_stabilized max setpoints for stabilized mode (in RPW rad, rad, rad/s)
   * @param max_setpts_acro max setpoints for acro mode (in RPW rad/s, rad/s, rad/s)
   * @param throttle_limits The min (index 0) and max (index 1) throttle values between 0-1
   */
  Setpoint(FlightMode flight_mode, std::array<float, 3> max_setpts_stabilized, std::array<float, 3> max_setpts_acro,
           std::array<float, 2> throttle_limits);

  /**
   * @brief Construct a new Setpoint object using a yaml node. The node must have all the parameters needed in the main
   * constructor
   *
   * @param config_params The yaml node with config params
   */
  Setpoint(const YAML::Node& config_params);

  /**
   * @brief Destroy the Setpoint object
   *
   */
  ~Setpoint();

  Setpoint() = delete;

  /**
   * @brief Construct a new Setpoint object using the move constructor
   *
   */
  Setpoint(Setpoint&&);

  /**
   * @brief Construct a new Setpoint object using the move assignment operator
   *
   * @return Setpoint&
   */
  Setpoint& operator=(Setpoint&&);

  // The setpoint object 'owns' communication with hardware so it is not copyable
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
  SetpointData get_setpoint_data();

  /**
   * @brief Set the Yaw Setpoint value to a user defined offset
   *
   * @param ref the reference value to set
   */
  void set_yaw_ref(float ref);

  /**
   * @brief Get the Min Throttle
   *
   * @return float
   */
  float get_min_throttle() const { return throttle_limits_[0]; }

 private:
  /**
   * @brief Loop which reads dsm data and makes it available to the user via GetSetpointData()
   *
   */
  void setpoint_manager();

  std::thread setpoint_thread_;                 //< Thread that reads dsm2 UART
  std::unique_ptr<std::mutex> setpoint_mutex_;  //< Mutex that protects setpoint_data_
  SetpointData setpoint_data_;                  //< Data to be shared with the user when called

  std::atomic<bool> is_running_;         //< Flag to indicate if the object is running vs shutting down
  std::atomic<bool> has_received_data_;  //< True if any dsm packet has been received

  // All configurable parameters
  FlightMode flight_mode_;                         //< The flight mode
  std::array<float, 3> max_setpoints_stabilized_;  //< Max orientation commands in [rad, rad, rad/s]
  std::array<float, 3> max_setpoints_acro_;        //< Max orientation commands in acro mode [rad/s, rad/s, rad/s]
  std::array<float, 2> throttle_limits_;           //< Min and max throttle limits
  // bool is_headless_mode_;  //< Not currently supported
};

}  // namespace flyMS
