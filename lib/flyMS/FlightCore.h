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

#include "flyMS/DigitalFilter.hpp"
#include "flyMS/gps.h"
#include "flyMS/imu/Imu.h"
#include "flyMS/mavlink_interface.h"
#include "flyMS/position_generator.h"
#include "flyMS/pru_client.h"
#include "flyMS/setpoint.h"
#include "flyMS/types/flight_mode.h"
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
   * @brief Perform all initialization tasks and start threads
   *
   * @return int
   */
  int init();

 private:
  /**
   * @brief Main control loop calculation for the flight stack. This function is registered to the Imu object as a
   * callback which gets invoked every time the IMU signals data is available to procoess
   *
   * @param imu_data_body
   * @return int
   */
  int flight_core(StateData &imu_data_body);

  /**
   * @brief Zeros the data stored in the PID controllers, helps reset the integrator
   *
   */
  void zero_pids();

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
  int console_print(const StateData &imu_data_body, const SetpointData &setpoint);

  // Variables for working with flyStereo
  bool flyStereo_running_ = false;
  bool flyStereo_streaming_data_ = false;
  float standing_throttle_ = 0.0f;
  float initial_yaw_ = 0.0f;
  PositionGenerator position_generator_;

  // Counter for number of timestamps at min throttle, used to detect landing
  // and reset the integrators in the PID controllers
  int integrator_reset_ = 0;

  Imu &imu_module_;                         //< Reference to Imu singleton object and Data struct from the imu manager
  ULog ulog_;                               //< Class to handle and write to the log file
  pruClient pru_client_;                    //< Object that communicates with the pru_handler process
  Setpoint setpoint_module_;                //< Object and Data struct from the setpoint manager
  MavlinkInterface mavlink_interface_;      //< Object to handle the I/O on the serial port
  PositionController position_controller_;  //< Controller for position when flying using flySetero
  gps gps_module_;                          //< Data struct from the gps manager
  GPS_data_t gps_;                          //< GPS Manager

  DigitalFilter roll_inner_PID_;
  DigitalFilter roll_outer_PID_;
  DigitalFilter pitch_inner_PID_;
  DigitalFilter pitch_outer_PID_;
  DigitalFilter yaw_PID_;
  DigitalFilter gyro_lpf_pitch_;
  DigitalFilter gyro_lpf_roll_;
  DigitalFilter gyro_lpf_yaw_;

  // Configurable parameters
  bool is_debug_mode_;                       //< Is running in debug mode (no output to Motors)
  FlightMode flight_mode_;                   //< The current flight mode
  std::array<float, 3> max_control_effort_;  //< Maximum values allowed to to exert in each euler angle DoF
  std::string log_filepath_;                 //< The filepath to the logging directory
  YAML::Node config_params_;                 //< A copy of the yaml configuration

  DigitalFilter *test_filter;
};

}  // namespace flyMS
