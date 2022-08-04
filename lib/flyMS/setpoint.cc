/**
 * @file setpoint.cpp
 * @brief Class for controlling the input reference angle to the inner loop controller
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/setpoint.h"

#include <chrono>
#include <iostream>

#include "flyMS/imu/Imu.h"
#include "rc/start_stop.h"
#include "spdlog/spdlog.h"

namespace flyMS {

static constexpr uint32_t SETPOINT_LOOP_FRQ = 40;
static constexpr uint32_t SETPOINT_LOOP_SLEEP_TIME_US = 1E6 / 40;
static constexpr float DEADZONE_THRESH = 0.05f;

Setpoint::Setpoint(bool is_debug_mode, FlightMode flight_mode, std::array<float, 3> max_setpts_stabilized,
                   std::array<float, 3> max_setpts_acro, std::array<float, 2> throttle_limits)
    : is_running_(false),
      has_received_data_(false),
      is_debug_mode_(is_debug_mode),
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

Setpoint::Setpoint(const YAML::Node& config_params)
    : Setpoint(config_params["debug_mode"].as<bool>(),
               static_cast<FlightMode>(config_params["flight_mode"].as<uint32_t>()),
               config_params["setpoint"]["max_setpoints_stabilized"].as<std::array<float, 3>>(),
               config_params["setpoint"]["max_setpoints_acro"].as<std::array<float, 3>>(),
               config_params["setpoint"]["throttle_limits"].as<std::array<float, 2>>()) {}

Setpoint::~Setpoint() {
  rc_dsm_cleanup();
  is_running_.store(false);
  if (setpoint_thread_.joinable()) {
    setpoint_thread_.join();
  }
}

void Setpoint::init() {
  int ret = rc_dsm_init();
  if (ret < 0) {
    throw std::invalid_argument("DSM failed to initialize");
  }

  is_running_.store(true);
  setpoint_thread_ = std::thread(&Setpoint::setpoint_manager, this);
}

void Setpoint::wait_for_data_packet() {
  while (!has_received_data_.load() && rc_get_state() != EXITING) {
    std::this_thread::sleep_for(std::chrono::microseconds(SETPOINT_LOOP_SLEEP_TIME_US));
  }
}

// Gets the data from the local thread. Returns zero if no new data is available
SetpointData Setpoint::get_setpoint_data() {
  std::lock_guard<std::mutex> lock(setpoint_mutex_);
  return setpoint_data_;
}

void Setpoint::setpoint_manager() {
  uint32_t dsm2_timeout_counter = 0;
  while (is_running_.load()) {
    if (rc_dsm_is_new_data()) {
      std::lock_guard<std::mutex> lock(setpoint_mutex_);

      // Set roll/pitch reference value
      // DSM2 Receiver is inherently positive to the left
      if (flight_mode_ == FlightMode::STABILIZED) {  // Stabilized Flight Mode
        setpoint_data_.euler_ref[0] = -rc_dsm_ch_normalized(2) * max_setpoints_stabilized_[0];
        setpoint_data_.euler_ref[1] = rc_dsm_ch_normalized(3) * max_setpoints_stabilized_[1];
      } else if (flight_mode_ == FlightMode::ACRO) {
        setpoint_data_.euler_ref[0] = -rc_dsm_ch_normalized(2) * max_setpoints_acro_[0];
        setpoint_data_.euler_ref[1] = rc_dsm_ch_normalized(3) * max_setpoints_acro_[1];
      } else {
        throw std::runtime_error("Error! Invalid flight mode");
      }

      // Set Yaw, RC Controller acts on Yaw velocity, save a history for integration
      // Apply the integration outside of current if statement, needs to run at 200Hz
      setpoint_data_.yaw_rate_ref[1] = setpoint_data_.yaw_rate_ref[0];
      setpoint_data_.yaw_rate_ref[0] = rc_dsm_ch_normalized(4) * max_setpoints_stabilized_[2];

      // Apply a deadzone to keep integrator from wandering
      if (std::abs(setpoint_data_.yaw_rate_ref[0]) < DEADZONE_THRESH) {
        setpoint_data_.yaw_rate_ref[0] = 0;
      }

      // Kill Switch
      setpoint_data_.kill_switch = rc_dsm_ch_normalized(5);

      // Auxillary Switch
      setpoint_data_.Aux[1] = setpoint_data_.Aux[0];
      setpoint_data_.Aux[0] = rc_dsm_ch_normalized(6);

      // Set the throttle
      setpoint_data_.throttle =
          (rc_dsm_ch_normalized(1) + 1.0) / 2.0 * (throttle_limits_[1] - throttle_limits_[0]) + throttle_limits_[0];

      // Finally Update the integrator on the yaw reference value
      setpoint_data_.euler_ref[2] =
          setpoint_data_.euler_ref[2] +
          (setpoint_data_.yaw_rate_ref[0] + setpoint_data_.yaw_rate_ref[1]) * 1 / (2 * SETPOINT_LOOP_FRQ);

      dsm2_timeout_counter = 0;
      if (!has_received_data_.load()) {
        has_received_data_.store(true);
      }
    } else {
      if (!is_debug_mode_) {
        // check to make sure too much time hasn't gone by since hearing the RC
        dsm2_timeout_counter++;
        if (dsm2_timeout_counter > 2 * SETPOINT_LOOP_FRQ) {  // If packet hasn't been received for 2 seconds
          spdlog::critical("Lost Connection with Remote!! Shutting Down Immediately!");
          rc_set_state(EXITING);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(SETPOINT_LOOP_SLEEP_TIME_US));
  }
}

void Setpoint::set_yaw_ref(float ref) { setpoint_data_.euler_ref[2] = ref; }

}  // namespace flyMS
