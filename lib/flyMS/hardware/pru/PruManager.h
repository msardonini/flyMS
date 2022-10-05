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
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include "flyMS/hardware/pru/pru_messages.h"
#include "flyMS/ipc/redis/RedisPublisher.h"
#include "flyMS/ipc/redis/RedisSubscriber.h"
#include "rc/servo.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

static constexpr int kNUM_PRU_CHANNELS = 4;
static constexpr int kCOMMAND_FRQ_HZ = 200;  // Hz
static constexpr auto kCOMMAND_THREAD_SLEEP_TIME = std::chrono::microseconds(1000000 / kCOMMAND_FRQ_HZ);
static const std::filesystem::path kPID_FILE_PRU(
    "/var/PruManager.pid");  //< PID file for PruManager, ensured only one instance is running

/**
 * @brief Object to handle ownership of the Programmable Realtime Unit (PRU) and what is allowed to send commands to it.
 * The PRU is used to send commands to the Electronic Speed Controllers (ESCs), which control the drone's motors.
 *
 * The PRU is primarily owned by this object, while it owns the PRU it commands zeros to the ESCs. This is because ESCs
 * will frequently complain if there is no signal sent to them.
 *
 * Other processes can request to use the PRU via PruRequster. This request is done via the PruRequester obect.
 * Internally, Redis publishers and subscribers are used for communication. If the PRU is not in use, the request is
 * granted. If the PRU is in use, the request is denied. The object that owns the PRU can return ownership to this
 * object, which is done in the destructor of PruRequester.
 *
 * This object is supposed to be instanted in a daemon, and kept alive while hardware is running.
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
                          {kredis_request_channel, kredis_release_channel}) {
    // Ensure there is only one instance of this program running system-wide
    if (!pid_handler(kPID_FILE_PRU)) {
      throw std::runtime_error("Could not create PID file for PruManager.");
    }
    redis_subscriber_.set_error_callback([](const sw::redis::Error &er) {});  // ignore timeouts, they are expected

    // Initialize the PRU hardware
    rc_servo_init();
    owns_pru_ = true;
    start_sending_zeros();
  }
  ~PruManager() {
    std::filesystem::remove(kPID_FILE_PRU);
    stop_sending_zeros();
  }

  // This object is not copyable or movable
  PruManager(const PruManager &) = delete;
  PruManager(PruManager &&) = delete;
  PruManager &operator=(const PruManager &) = delete;
  PruManager &operator=(PruManager &&) = delete;

 private:
  /**
   * @brief Function to handle the PID file for this program. This ensures that only one instance of this program is
   * running system-wide. This will create a PID file at the location provided. If the PID file already exists and there
   * is a process with that PID, this function will return false. Otherwise, it will create the PID file and return
   * true.
   *
   * @param pid_file Path to the PID file to create.
   * @return true The pid file was created successfully
   * @return false There was an error creating the PID file
   */
  bool pid_handler(const std::filesystem::path &pid_file_path) {
    if (std::filesystem::exists(pid_file_path)) {
      std::ifstream pid_file(pid_file_path);
      int pid;
      pid_file >> pid;
      pid_file.close();

      // Check if there is a process running with this PID
      if (kill(pid, 0) == 0) {
        spdlog::error("Cannot start PruManager, another instance is already running.");
        return false;
      } else {
        spdlog::warn("Removing stale PID file for PruManager.");
        if (!std::filesystem::remove(pid_file_path)) {
          spdlog::error("Could not remove stale PID file for PruManager. Do you have permission?");
          return false;
        }
      }
    }

    // Create the PID file
    std::ofstream pid_file(pid_file_path);
    pid_file << getpid();
    pid_file.close();

    if (!pid_file) {
      spdlog::error("Could not create the PID file for PruManager. Do you have permission?");
      return false;
    } else {
      return true;
    }
  }

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
   * @brief Stops the thread that sends zeros to the PRU
   *
   */
  void stop_sending_zeros() {
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
  }

  /**
   * @brief Starts the thread that sends zeros to the PRU
   *
   */
  void start_sending_zeros() {
    spdlog::debug("Starting thread to send zeros to PRU.");
    if (!owns_pru_) {
      spdlog::warn("Instructed to start sending zeros, without ownership of the PRU.");
      return;
    }
    is_running_zero_sender_ = true;
    send_zero_thread_ = std::thread(&PruManager::send_zeros, this);
  }

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
        stop_sending_zeros();
        rc_servo_cleanup();
        owns_pru_ = false;
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
        owns_pru_ = true;
        rc_servo_init();
        start_sending_zeros();
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

  RedisSubscriber redis_subscriber_;   //< Redis subscriber to listen for requests
  RedisPublisher redis_publisher_;     //< Redis publisher to send responses
  int process_id_sender_ = -1;         //< The process ID of the requesting process
  std::thread send_zero_thread_;       //< Thread for sending zeros to the PRU
  std::atomic<bool> owns_pru_{false};  //< Flag to indicate if this object owns the PRU. This is also used to send zeros
                                       // to the PRU when nothing is connected to it
  std::atomic<bool> is_running_zero_sender_{false};  //< Flag to indicate if the zero sender thread is running
};

}  // namespace flyMS
