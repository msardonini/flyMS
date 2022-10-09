/**
 * @file PruManager.h
 * @author Mike Sardonini (msaronini@gmail.com)
 * @brief PruManager class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <signal.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include "flyMS/hardware/pru/pru_messages.h"
#include "flyMS/ipc/redis/RedisPublisher.h"
#include "flyMS/ipc/redis/RedisSubscriber.h"
#include "flyMS/util/pid_file.h"
#include "rc/servo.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

static constexpr int kNUM_PRU_CHANNELS = 4;
static constexpr int kCOMMAND_FRQ_HZ = 200;  // Hz
static constexpr auto kCOMMAND_THREAD_SLEEP_TIME = std::chrono::microseconds(1000000 / kCOMMAND_FRQ_HZ);
static constexpr auto kPID_MONITOR_SLEEP_TIME = std::chrono::seconds(1);
static const std::filesystem::path kPID_FILE_PRU(
    "/var/PruManager.pid");  //< PID file for PruManager, ensured only one instance is running

/**
 * @brief Handle the ownership of the Programmable Realtime Unit (PRU) and what is allowed to send commands to it.
 * The PRU is used to send commands to the Electronic Speed Controllers (ESCs), which control the drone's motors.
 *
 * The PRU is primarily owned by this object, while it owns the PRU it commands zeros to the ESCs. This is because ESCs
 * will frequently complain if there is no signal sent to them.
 *
 * Other processes can request to use the PRU via PruRequester. This request is done via the PruRequester object.
 * Internally, Redis publishers and subscribers are used for communication. If the PRU is not in use, the request is
 * granted. If the PRU is in use, the request is denied. The object that owns the PRU can return ownership to this
 * object, which is done in the destructor of PruRequester.
 *
 * This object is supposed to be instantiated in a daemon, and kept alive while hardware is running.
 *
 * Example usage is:
 * ```
 * PruManager pru_manager;  // Internals are all started in the constructor
 * while (hardware_is_running) {
 *   sleep(1);
 * }
 * ```

 */
class PruManager {
 public:
  /**
   * @brief Construct a new Pru Handler object. The constructor does the following:
   * 1. Ensures only one instance of this object is running system wide with a PID file
   * 2. Sets up the Redis subscribers and publishers, subscribes to channels
   * 3. Initializes hardware and starts commanding zeros while this object owns the PRU
   */
  PruManager()
      : redis_subscriber_(std::bind(&PruManager::on_message, this, std::placeholders::_1, std::placeholders::_2),
                          {kredis_request_channel, kredis_release_channel}),
        pid_file_(kPID_FILE_PRU) {
    // Ensure there is only one instance of this program running system-wide
    redis_subscriber_.set_error_callback([](const sw::redis::Error &er) {});  // ignore timeouts, they are expected

    // Initialize the PRU hardware
    take_pru_ownership();
  }
  ~PruManager() {
    std::filesystem::remove(kPID_FILE_PRU);
    release_pru_ownership(false);
  }

  // This object is not copyable or movable
  PruManager(const PruManager &) = delete;
  PruManager(PruManager &&) = delete;
  PruManager &operator=(const PruManager &) = delete;
  PruManager &operator=(PruManager &&) = delete;

 private:
  /**
   * @brief Sends zeros to the PRU. This is intended to be used when nothing is connected to the PRU. This prevents ESCs
   * from chirping / complaining that they receive no signal while the flyMS program is not running
   *
   */
  void send_zeros() {
    while (is_running_zero_sender_) {
      for (auto i = 0; i < kNUM_PRU_CHANNELS; i++) {
        rc_servo_send_esc_pulse_normalized(i + 1, 0.0);
      }
      std::this_thread::sleep_for(kCOMMAND_THREAD_SLEEP_TIME);
    }
  }

  /**
   * @brief Starts the thread that sends zeros to the PRU
   *
   */
  void start_sending_zeros() {}

  static void on_timeout(const sw::redis::Error &err) {}

  /**
   * @brief Callback function for when a message is received on the Redis subscriber. This function handles the logic to
   * passing ownership of the PRU to requesting processes. Only one process can own the PRU at a time, and it should
   * return ownership when it is done.
   *
   * When a request message is received, this function generates a response message depending on the state of this
   * object.
   *
   * @param channel The channel the message was received on
   * @param message The message received. It is expected that this message is in a YAML string format, can be
   *                deserialized. Messages from pru_message.h are expected.
   */
  void on_message(std::string_view channel, std::string_view message) {
    if (channel == std::string_view(kredis_request_channel)) {
      PruResponse response;
      auto pru_request = from_yaml<PruRequest>(YAML::Load(message.data()));
      if (owns_pru_) {
        process_id_sender_ = pru_request.pid;
        spdlog::info("Received request to own PRU from process with PID {}", pru_request.pid);
        release_pru_ownership();
        response.success = true;
        response.reason = "success";
      } else {
        spdlog::warn("Received request when someone else owns PRU");
        response.success = false;
        response.reason = "PRU already in use";
      }
      redis_publisher_.publish(kredis_request_response_channel, to_yaml(response));
    } else if (channel == std::string_view(kredis_release_channel)) {
      auto request = from_yaml<PruRequest>(YAML::Load(message.data()));
      PruResponse response;
      if (request.pid == process_id_sender_) {
        spdlog::info("Received request to release PRU from process with PID {}", request.pid);
        take_pru_ownership();
        process_id_sender_ = -1;
        response.success = true;
        response.reason = "success";
      } else {
        spdlog::warn("Process ID does not match the one that requested access");
        response.success = false;
        response.reason = "Process ID does not match the one that requested access";
      }
      redis_publisher_.publish(kredis_release_response_channel, to_yaml(response));
    } else {
      spdlog::error("Received message on unknown channel: {}", channel);
    }
  }

  /**
   * @brief Function to transistion states from not owning the PRU to owning the PRU
   *
   */
  void take_pru_ownership() {
    owns_pru_ = true;
    rc_servo_init();
    spdlog::debug("Starting thread to send zeros to PRU.");
    if (!owns_pru_) {
      spdlog::warn("Instructed to start sending zeros, without ownership of the PRU.");
      return;
    }
    is_running_zero_sender_ = true;
    send_zero_thread_ = std::thread(&PruManager::send_zeros, this);
  }

  /**
   * @brief Function to transition states from owning the PRU to not owning the PRU
   *
   * @param start_pid_monitor optionally start the PID monitor thread
   */
  void release_pru_ownership(bool start_pid_monitor = true) {
    spdlog::debug("Stopping zero sender thread.");
    if (!owns_pru_) {
      spdlog::warn("Instructed to stop sending zeros to PRU, but PRU is not owned.");
      return;
    }

    is_running_zero_sender_ = false;
    if (send_zero_thread_.joinable()) {
      send_zero_thread_.join();
    }
    spdlog::debug("Stopped zero sender thread.");
    rc_servo_cleanup();
    owns_pru_ = false;

    // Start the pid monitor thread to make sure the requesting process is running while it owns PRU. First make sure a
    // previous thread has been joined
    if (pid_monitor_thread_.joinable()) {
      is_running_pid_monitor_ = false;
      pid_monitor_thread_.join();
    }
    if (start_pid_monitor) {
      spdlog::debug("Starting monitor thread.");
      pid_monitor_thread_ = std::thread(&PruManager::pid_monitor_thread, this);
    }
  }

  /**
   * @brief Monitors the process that requested the ownership of the PRU. If the process dies without releasing the
   *        PRU, this thread will log an error and release the PRU.
   *
   */
  void pid_monitor_thread() {
    is_running_pid_monitor_ = true;
    while (is_running_pid_monitor_) {
      if (kill(process_id_sender_, 0) != 0) {
        spdlog::error("PID {} has shut down before releasing PRU. Releasing PRU manually.", process_id_sender_);
        take_pru_ownership();
        is_running_pid_monitor_ = false;
      }
      std::this_thread::sleep_for(kPID_MONITOR_SLEEP_TIME);
    }
  }

  RedisSubscriber redis_subscriber_;   //< Redis subscriber to listen for requests
  RedisPublisher redis_publisher_;     //< Redis publisher to send responses
  int process_id_sender_ = -1;         //< The process ID of the requesting process
  std::thread send_zero_thread_;       //< Thread for sending zeros to the PRU
  std::thread pid_monitor_thread_;     //< Thread for sending zeros to the PRU
  std::atomic<bool> owns_pru_{false};  //< Flag to indicate if this object owns the PRU. This is also used to send zeros
                                       // to the PRU when nothing is connected to it
  std::atomic<bool> is_running_zero_sender_{false};  //< Flag to indicate if the zero sender thread is running
  std::atomic<bool> is_running_pid_monitor_{false};  //< Flag to indicate if the zero sender thread is running

  PidFile pid_file_;  //< PidFile object to monitor unique running instances of this program
};

}  // namespace flyMS
