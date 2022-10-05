/**
 * @file FlightCore.cc
 * @brief Main control loop for the flyMS program
 *
 * @author Mike Sardonini
 * @date 10/15/2018
 */

#include "flyMS/FlightCore.h"

#include <sys/stat.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "flyMS/hardware/RemoteController.h"
#include "flyMS/util/debug_mode.h"
#include "rc/start_stop.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace flyMS {

void FlightCore::zero_pids() {
  roll_outer_PID_.zero_values();
  roll_inner_PID_.zero_values();
  pitch_outer_PID_.zero_values();
  pitch_inner_PID_.zero_values();
  yaw_PID_.zero_values();
}

FlightCore::FlightCore(Setpoint &&setpoint, PositionController &&position_controller, const YAML::Node &config_params)
    : imu_module_(Imu::getInstance()),
      ulog_(),
      setpoint_module_(std::move(setpoint)),
      position_controller_(std::move(position_controller)),
      config_params_(config_params) {
  flight_mode_ = static_cast<FlightMode>(config_params["flight_mode"].as<uint32_t>());
  log_filepath_ = config_params["log_filepath"].as<std::string>();

  YAML::Node controller = config_params["controller"];
  max_control_effort_ = controller["max_control_effort"].as<std::array<float, 3>>();
}

FlightCore::FlightCore(const YAML::Node &config_params)
    : FlightCore(Setpoint(config_params["setpoint"]), PositionController(config_params["position_controller"]),
                 config_params) {}

bool FlightCore::init() {
  // Create a file for logging and initialize our file logger
  init_logging(log_filepath_);

  // TODO: improve the logic to decide whether to initialize the object or not
  if (config_params_["mavlink_interface"]["enable"].as<bool>()) {
    mavlink_subscriber_ = std::make_unique<MavlinkRedisSubQueue>(kFLY_STEREO_CHANNEL);
    odometry_queue_ = mavlink_subscriber_->register_message_with_queue<mavlink_odometry_t, MAVLINK_MSG_ID_ODOMETRY>(
        &mavlink_msg_odometry_decode);
  }

  // Blocks execution until a packet is received
  if constexpr (!kDEBUG_MODE) {
    setpoint_module_.wait_for_data_packet();

    // Command the state variable be set to EXITING if the RC loses connection
    RemoteController::get_instance().packet_loss_callback([]() { rc_set_state(EXITING); });
  }
  // Tell the system that we are running if we are in the predicted UNINITIALIZED state. Else shutdown
  if (rc_get_state() == UNINITIALIZED) {
    rc_set_state(RUNNING);
  } else {
    rc_set_state(EXITING);
  }

  // Generate the PID controllers and low-pass filters
  YAML::Node controller = config_params_["controller"];
  YAML::Node filters = config_params_["filters"];
  double pid_lpf_const = controller["pid_LPF_const_sec"].as<double>();
  roll_inner_PID_ = generate_pid(controller["roll_PID_inner"].as<std::array<double, 3>>(), pid_lpf_const, LOOP_DELTA_T);
  roll_outer_PID_ = generate_pid(controller["roll_PID_outer"].as<std::array<double, 3>>(), pid_lpf_const, LOOP_DELTA_T);
  pitch_outer_PID_ =
      generate_pid(controller["pitch_PID_outer"].as<std::array<double, 3>>(), pid_lpf_const, LOOP_DELTA_T);
  pitch_inner_PID_ =
      generate_pid(controller["pitch_PID_inner"].as<std::array<double, 3>>(), pid_lpf_const, LOOP_DELTA_T);
  yaw_PID_ = generate_pid(controller["yaw_PID"].as<std::array<double, 3>>(), pid_lpf_const, LOOP_DELTA_T);

  auto lpf_num = filters["imu_lpf_num"].as<std::vector<double>>();
  auto lpf_den = filters["imu_lpf_den"].as<std::vector<double>>();
  gyro_lpf_roll_ = DigitalFilter(lpf_num, lpf_den);
  gyro_lpf_pitch_ = DigitalFilter(lpf_num, lpf_den);
  gyro_lpf_yaw_ = DigitalFilter(lpf_num, lpf_den);

  // Request access to use the PRU, which we use to send commands to ESCs
  if (!pru_requester_.request_access()) {
    spdlog::error("Failed to get access to PRU");
    return false;
  }
  // Start the flight program
  std::function<void(StateData &)> func = std::bind(&FlightCore::flight_core, this, std::placeholders::_1);
  imu_module_.init(config_params_["imu_params"], func);

  return true;
}

void FlightCore::flight_core(StateData &imu_data_body) {
  /************************************************************************
   *       Check the Mavlink Interface for New Visual Odometry Data
   ************************************************************************/
  Eigen::Vector3f setpoint_orientation;
  if (mavlink_subscriber_ && odometry_queue_) {
    // Check if there is a new packet

    // TODO: handle navigation data properly with the position controller
    if (odometry_queue_->size() > 0) {
      mavlink_odometry_t odometry_packet = odometry_queue_->pop_front();
    }

    // if (is_valid) {
    //   spdlog::warn("vio x: {}, vio y: {}", vio.position[0], vio.position[1]);
    //   position_controller_.ReceiveVio(vio);
    //   std::tie(setpoint_orientation, vio_yaw) = position_controller_.GetSetpoint();

    //   float log_setpoint[4] = {setpoint_orientation(0), setpoint_orientation(1), vio_yaw, setpoint_orientation(2)};
    //   float quat_setpoint[4] = {vio.quat.w(), vio.quat.x(), vio.quat.y(), vio.quat.z()};

    //   // Log the VIO Data
    //   ULogPosCntrlMsg vio_log_msg(imu_data_body.timestamp_us, vio.position.data(), vio.velocity.data(),
    //   quat_setpoint,
    //                               log_setpoint);
    //   ulog_.write_msg(vio_log_msg);
    //   flyStereo_streaming_data_ = true;
    // }
  }

  /************************************************************************
   *                          Get Setpoint Data                            *
   ************************************************************************/
  auto setpoint = setpoint_module_.get_setpoint_data();

  // If we have commanded a switch in Aux, activate the perception system
  if (mavlink_subscriber_) {
    if (setpoint.Aux[0] < 0.1 && setpoint.Aux[1] > 0.9) {
      // Transition to start flyStereo
      // mavlink_subscriber_->SendStartCommand();

      // Assume that the current throttle value will be an average value to keep
      // altitude
      standing_throttle_ = setpoint.throttle;
      initial_yaw_ = imu_data_body.euler[2];

      // Make sure our first setpoint is zero
      flyStereo_running_ = true;
    } else if (setpoint.Aux[0] > 0.9 && setpoint.Aux[1] < 0.1) {
      flyStereo_running_ = false;
      flyStereo_streaming_data_ = false;
      // mavlink_subscriber_->SendStopCommand();

      // Reset the filters in the Position controller
      position_controller_.ResetController();
      setpoint_orientation = Eigen::Vector3f::Zero();
      position_generator_.ResetCounter();
    }
  }

  // Apply orientation commands from the position controller if running
  if (flyStereo_running_ && flyStereo_streaming_data_) {
    setpoint.euler_ref[0] = setpoint_orientation(0);
    setpoint.euler_ref[1] = setpoint_orientation(1);

    setpoint.throttle = setpoint_orientation(2) + standing_throttle_;
    // setpoint.euler[2] = vio_yaw - imu_data_body.euler[2] + initial_yaw_;

    // Apply the position setpoint
    Eigen::Vector3f pos_ref;
    position_generator_.GetPosition(&pos_ref);
    position_controller_.SetReferencePosition(pos_ref);
  }

  std::array<float, 3> u_euler;  //< The amount of controller effort (-1 to 1) to give in each euler angle [RPY]
  /************************************************************************
   *                         Roll Controller                               *
   ************************************************************************/
  float droll_setpoint;
  if (flight_mode_ == FlightMode::STABILIZED) {
    droll_setpoint = roll_outer_PID_.update_filter(setpoint.euler_ref[0] - imu_data_body.euler[0]);
  } else if (flight_mode_ == FlightMode::ACRO) {  // Acro mode
    droll_setpoint = setpoint.euler_ref[0];
  } else {
    throw std::runtime_error("[FlightCore] Error! Invalid flight mode. Shutting down now");
  }

  imu_data_body.eulerRate[0] = gyro_lpf_roll_.update_filter(imu_data_body.eulerRate[0]);
  u_euler[0] = roll_inner_PID_.update_filter(droll_setpoint - imu_data_body.eulerRate[0]);
  u_euler[0] = std::clamp(u_euler[0], -max_control_effort_[0], max_control_effort_[0]);

  /************************************************************************
   *                         Pitch Controller                              *
   ************************************************************************/
  float dpitch_setpoint;
  if (flight_mode_ == FlightMode::STABILIZED) {  // Stabilized Flight Mode
    dpitch_setpoint = pitch_outer_PID_.update_filter(setpoint.euler_ref[1] - imu_data_body.euler[1]);
  } else if (flight_mode_ == FlightMode::ACRO) {  // Acro mode
    dpitch_setpoint = setpoint.euler_ref[1];
  } else {
    throw std::runtime_error("[FlightCore] Error! Invalid flight mode. Shutting down now");
  }

  imu_data_body.eulerRate[1] = gyro_lpf_pitch_.update_filter(imu_data_body.eulerRate[1]);
  u_euler[1] = pitch_inner_PID_.update_filter(dpitch_setpoint - imu_data_body.eulerRate[1]);
  u_euler[1] = std::clamp(u_euler[1], -max_control_effort_[1], max_control_effort_[1]);

  /************************************************************************
   *                          Yaw Controller                              *
   ************************************************************************/
  imu_data_body.eulerRate[2] = gyro_lpf_yaw_.update_filter(imu_data_body.eulerRate[2]);
  u_euler[2] = yaw_PID_.update_filter(setpoint.euler_ref[2] - imu_data_body.euler[2]);
  // Apply a saturation filter
  u_euler[2] = std::clamp(u_euler[2], -max_control_effort_[2], max_control_effort_[2]);

  /************************************************************************
   *                  Reset the Integrators if Landed                      *
   ************************************************************************/
  if (setpoint.throttle < setpoint_module_.get_min_throttle() + .01) {
    integrator_reset_++;
  } else {
    integrator_reset_ = 0;
  }

  // if landed for 2 seconds, reset integrators and Yaw error
  if (integrator_reset_ > 2 / LOOP_DELTA_T) {
    setpoint.euler_ref[2] = imu_data_body.euler[2];
    setpoint_module_.set_yaw_ref(imu_data_body.euler[2]);
    zero_pids();
  }

  /************************************************************************
   *  Mixing
   *
   *                          CCW 1    2 CW       Body Frame:
   *                               \ /               X
   *                               / \             Y_|
   *                           CW 3   4 CCW              Z UP
   *
   ************************************************************************/
  // The amount of power (0 - 1) to give to each motor
  std::array<float, 4> u;
  u[0] = setpoint.throttle + u_euler[0] - u_euler[1] - u_euler[2];
  u[1] = setpoint.throttle - u_euler[0] - u_euler[1] + u_euler[2];
  u[2] = setpoint.throttle + u_euler[0] + u_euler[1] + u_euler[2];
  u[3] = setpoint.throttle - u_euler[0] + u_euler[1] - u_euler[2];

  /************************************************************************
   *             Check Output Ranges, if outside, adjust                 *
   ************************************************************************/
  static constexpr float smallest_value = 0.f;
  float largest_value = 1.f;
  for (int i = 0; i < 4; i++) {
    if (u[i] > largest_value) {
      largest_value = u[i];
    }

    if (u[i] < smallest_value) u[i] = 0;
  }

  // if upper saturation would have occurred, reduce all outputs evenly
  if (largest_value > 1) {
    float offset = largest_value - 1;
    for (int i = 0; i < 4; i++) u[i] -= offset;
  }

  /************************************************************************
   *                  Send Commands to ESCs                         *
   ************************************************************************/
  if constexpr (!kDEBUG_MODE) {
    pru_requester_.send_command(u);
  }

  /************************************************************************
   *             Check the kill Switch and Shutdown if set               *
   ************************************************************************/
  if constexpr (!kDEBUG_MODE) {
    if (setpoint.kill_switch < 0.5) {
      spdlog::info("\nKill Switch Hit! Shutting Down\n");
      rc_set_state(EXITING);
    }
  }

  // Print some stuff to the console in debug mode
  if constexpr (kDEBUG_MODE) {
    // console_print(imu_data_body, setpoint, u);
  }

  /************************************************************************
   *               Log Important Flight Data For Analysis              *
   ************************************************************************/
  struct ULogFlightMsg flight_msg {
    imu_data_body.timestamp_us, imu_data_body, setpoint, u, u_euler
  };
  ulog_.write_msg(flight_msg);
}

int FlightCore::console_print(const StateData &imu_data_body, const SetpointData &setpoint,
                              const std::array<float, 4> &u) {
  //  spdlog::info("time {:3.3f} ", control->time);
  //  spdlog::info("Alt_ref {:3.1f} ",control->alt_ref);
  // spdlog::info("U1: {:2.2f}, U2: {:2.2f}, U3: {:2.2f}, U4: {:2.2f} ", u[0], u[1], u[2], u[3]);
  // spdlog::info("Aux {:2.1f} ", setpoint.Aux[0]);
  //  spdlog::info("function: {}",rc_get_dsm_ch_normalized(6));
  //  spdlog::info("num wraps {} ",control->num_wraps);
  spdlog::info(" Throt {:2.2f}, Roll_ref {:2.2f}, Pitch_ref {:2.2f}, Yaw_ref {:2.2f} KS {:2.2f} ", setpoint.throttle,
               setpoint.euler_ref[0], setpoint.euler_ref[1], setpoint.euler_ref[2], setpoint.kill_switch);
  // spdlog::info("Roll {:1.2f}, Pitch {:1.2f}, Yaw {:2.3f}", imu_data_body.euler[0], imu_data_body.euler[1],
  //              imu_data_body.euler[2]);
  // spdlog::info("droll {:1.2f}, dpitch {:1.2f}, dyaw {:2.3f}", imu_data_body.eulerRate[0], imu_data_body.eulerRate[1],
  //              imu_data_body.eulerRate[2]);
  //  spdlog::info(" Mag X {:4.2f}",control->mag[0]);
  //  spdlog::info(" Mag Y {:4.2f}",control->mag[1]);
  //  spdlog::info(" Mag Z {:4.2f}",control->mag[2]);
  // spdlog::info(" Accel X {:4.2f}",control->accel[0]);
  // spdlog::info(" Accel Y {:4.2f}",control->accel[1]);
  // spdlog::info(" Accel Z {:4.2f}",control->accel[2]);
  //   spdlog::info(" Pos N {:2.3f} ", control->ekf_filter.output.ned_pos[0]);
  //  spdlog::info(" Pos E {:2.3f} ", control->ekf_filter.output.ned_pos[1]);
  //  spdlog::info(" Pos D {:2.3f} ", control->ekf_filter.output.ned_pos[2]);
  // spdlog::info(" DRoll {:1.2f}, DPitch {:1.2f}, DYaw {:2.3f}",
  // imu_data_body.eulerRate[0],
  //   imu_data_body.eulerRate[1], imu_data_body.eulerRate[2]);
  // spdlog::info("uroll {:2.3f}, upitch {:2.3f}, uyaw {:2.3f}", u_euler[0],
  // u_euler[1],
  //   u_euler[2]);
  //  spdlog::info(" GPS pos lat: {:2.2f}", control->GPS_data.pos_lat);
  //  spdlog::info(" GPS pos lon: {:2.2f}", control->GPS_data.pos_lon);
  //  spdlog::info(" HDOP: {}", control->GPS_data.HDOP);
  //  spdlog::info("Baro Alt: {} ",control->baro_alt);
  return 0;
}

void FlightCore::init_logging(const std::filesystem::path &log_location) {
  auto log_dir = ULog::generate_incremented_run_dir(log_location);

  // Initialize spdlog
  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  int max_bytes = 1048576 * 20;  // Max 20 MB
  int max_files = 20;
  sinks.push_back(
      std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_dir / "console_log.txt", max_bytes, max_files));
  auto flyMS_log = std::make_shared<spdlog::logger>("flyMS_log", std::begin(sinks), std::end(sinks));

  // Register loggers to be global logger
  flyMS_log->set_level(spdlog::level::trace);
  spdlog::register_logger(flyMS_log);
  spdlog::set_default_logger(flyMS_log);

  // Initialize Ulog
  ulog_.init(log_dir / "logger.ulg");
}

}  // namespace flyMS
