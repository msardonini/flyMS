/**
 * @file mavlink_interface.h
 * @author Mike Sardonini
 * @brief Declaration of MavlinkInterface class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <unistd.h>

#include <atomic>
#include <thread>
#include <unordered_map>
#include <utility>

#include "flyMS/types/state_data.h"
#include "flyMS/types/vio_data.h"
#include "mavlink_v2/common/mavlink.h"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

class MavlinkInterface {
 public:
  // Delete Unrelated constructors
  MavlinkInterface() = delete;
  MavlinkInterface(const MavlinkInterface &) = delete;
  MavlinkInterface(MavlinkInterface &&) = delete;
  MavlinkInterface &operator()(const MavlinkInterface &) = delete;
  MavlinkInterface &operator()(MavlinkInterface &&) = delete;

  /**
   * @brief Construct a new Mavlink Interface object
   *
   * @param serial_dev The file path to the serial device
   */
  MavlinkInterface(const std::string &serial_dev);

  /**
   * @brief Construct a new Mavlink Interface object
   *
   * @param input_params Input configuration with a YAML::Node. The yaml node must have fields for all arguments in the
   * primary constructor.
   */
  MavlinkInterface(const YAML::Node input_params);

  ~MavlinkInterface();

  operator bool() { return is_running_.load(); }

  /**
   * @brief Register a callback function to process a mavlink message with a unique ID
   *
   * @param callback pair first first: unique ID of the mavlinkg message to process, second: callback function
   */
  void register_callback(std::pair<uint32_t, std::function<void(mavlink_message_t &)>> callback) {
    mavlink_message_handlers_.insert(callback);
  }

  /**
   * @brief Sends the IMU message over the mavlink serial bus
   *
   * @param imu_state The data to send
   * @return int 0 on success, -1 on failure
   */
  int send_imu_message(const StateData &imu_state);

  /**
   * @brief Sends the start command, which signals to start measuring VIO in flyStereo
   *
   * @return int 0 on success, -1 on failure
   */
  int send_start_command();

  /**
   * @brief Sends the shutdown command, which signals flyStereo to stop measuring
   *
   * @return int 0 on success, -1 on failure
   */
  int send_stop_command();

  /**
   * @brief Get the Visual Inertial Odometry (Vio) Data object
   *
   * @param vio the pointer to insert data to
   * @return true if new data has been received and vio is populated, false otherwise
   */
  bool get_vio_data(VioData *vio);

  template <typename ContainerT, typename EncodeFuncionT>
  void send_mavlink_message(const ContainerT &container, EncodeFuncionT encode_function) {
    mavlink_message_t msg;
    uint8_t buf[1024];
    encode_function(0, 1, &msg, &container);
    auto len = mavlink_msg_to_send_buffer(buf, &msg);
    std::lock_guard<std::mutex> lock(serial_send_mutex_);
    if (write(serial_dev_, buf, len) < len) {
      spdlog::error("[MavlinkInterface] write() did not succeed");
    }
  }

 private:
  /**
   * @brief Initialize the interface, opens the serial port and starts the threads
   *
   * @return int
   */
  int init();

  /**
   * @brief Thread the reads mavlink serial data
   *
   */
  void serial_read_thread();

  std::atomic<bool> is_running_;    //< Flag to tell if we are still operating
  int serial_dev_;                  //< file descriptor for the serial port
  std::string serial_dev_file_;     //< Filename of the serial port in the device tree
  std::thread serial_read_thread_;  //< Thread that reads the serial port for new data
  std::mutex serial_send_mutex_;    //< Mutex which prevents us from writing to the serial port at the same time
  std::mutex vio_mutex_;            //< mutex for the vio data object
  VioData vio_;                     //< contains data from the visual inertial odometry system (flyStereo)
  bool is_new_vio_data_ = false;    //< Flag if via data is new, i.e. hasn't been read yet
  std::unordered_map<uint32_t, std::function<void(mavlink_message_t &)>>
      mavlink_message_handlers_;  //< map of message types to functions to handle them)
};

}  // namespace flyMS
