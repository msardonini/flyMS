/**
 * @file FlightCore.h
 * @brief Header for the FlightCore object
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */
#pragma once

// System Includes
#include <array>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "flyMS/gps.h"
#include "flyMS/imu/Imu.h"
#include "flyMS/mavlink_interface.h"
#include "flyMS/position_generator.h"
#include "flyMS/pru_client.h"
#include "flyMS/setpoint.h"
#include "flyMS/types/flight_filters.h"
#include "flyMS/types/state_data.h"
#include "flyMS/ulog/ulog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

class FlightCore {
 public:
  /**
   * @brief Construct a new Flight Core object
   *
   * @param input_params Input parameters needed for flight
   */
  FlightCore(const YAML::Node &input_params);
  ~FlightCore() = default;

  /**
   * @brief Main control loop calculation for the flight stack. This function is registered to the Imu object as a
   * callback which gets invoked every time the IMU signals data is available to procoess
   *
   * @param imu_data_body
   * @return int
   */
  int flight_core(StateData &imu_data_body);

  /**
   * @brief Perform all initialization tasks and start threads
   *
   * @return int
   */
  int StartupRoutine();

 private:
  /**
   * @brief Checks the output signals before sending commands to motors. If any of the signals are too high (>1), all
   * channels are reduced evenly
   *
   * @param u The vector of signals going to ESCs/motors
   */
  void CheckOutputRange(std::array<float, 4> &u);

  /**
   * @brief Initializes all logging for flyMS. First, it creates a unique log directory 'runXXX' (datatime not used
   * because it is typically not synced). Then in initialized spdlog, then ulog
   *
   * @param log_dir locaation to store log directories
   */
  void init_logging(const std::string &log_location);

  /**
   * @brief Debug function which writes system into to the console. Typically only called in debug mode
   *
   * @param imu_data_body imu data in body frame
   * @param setpoint setpoint data
   * @return int
   */
  int ConsolePrint(const StateData &imu_data_body, const SetpointData &setpoint);

  // Variables for working with flyStereo
  bool flyStereo_running_ = false;
  bool flyStereo_streaming_data_ = false;
  float standing_throttle_ = 0.0f;
  float initial_yaw_ = 0.0f;
  PositionGenerator position_generator_;

  // Counter for number of timestamps at min throttle, used to detect landing
  // and reset the integrators in the PID controllers
  int integrator_reset_ = 0;

  // Reference to Imu singleton object and Data struct from the imu manager
  Imu &imu_module_;

  // Class to handle and write to the log file
  ULog ulog_;

  // Classes for all the functions of the program
  pruClient pru_client_;

  // Object and Data struct from the setpoint manager
  std::unique_ptr<Setpoint> setpoint_module_;

  // Object and Data struct from the gps manager
  gps gps_module_;
  GPS_data_t gps_;

  // Object to handle the I/O on the serial port
  std::unique_ptr<MavlinkInterface> mavlink_interface_;

  // Containers for controller's output
  std::array<float, 4> u_euler_;
  std::array<float, 4> u_;

  // Configurable parameters
  int flight_mode_;
  std::array<float, 3> max_control_effort_;
  bool is_debug_mode_;
  std::string log_filepath_;

  std::unique_ptr<FlightFilters> flight_filters_;

  YAML::Node config_params_;
};

}  // namespace flyMS
