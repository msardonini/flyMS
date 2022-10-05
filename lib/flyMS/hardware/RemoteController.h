#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace flyMS {

// Verbose names for the physical channels of the remote controller
static constexpr uint32_t kRC_THROTTLE_INDEX = 0;
static constexpr uint32_t kRC_ROLL_INDEX = 1;
static constexpr uint32_t kRC_PITCH_INDEX = 2;
static constexpr uint32_t kRC_YAW_INDEX = 3;
static constexpr uint32_t kRC_KILL_SWITCH_INDEX = 4;
static constexpr uint32_t kRC_FLIGHT_MODE_INDEX = 5;

/**
 * @brief Get commands from the remote controller via the DSM2 protocol. This class is a singleton, because there is
 * only once piece of hardware for reading the remote controller. Under the hood, the rc_dsm module from the
 * robotics_cape library is used.
 *
 * Example Usage:
 * @code
 * auto& rc = RemoteController::get_instance();
 * while(true) {
 *   auto rc_data = rc.get_channel_values();
 *   float throttle = rc_data[kRC_THROTTLE_INDEX];
 *   float roll = rc_data[kRC_ROLL_INDEX];
 *   // ...
 * }
 * @endcode
 */
class RemoteController {
 public:
  // Delete copy ctor and copy assignment operator
  RemoteController(const RemoteController &) = delete;
  RemoteController &operator=(const RemoteController &) = delete;

  /**
   * @brief Get the instance of the singleton RemoteController object
   *
   * @return RemoteController&
   */
  static RemoteController &get_instance();

  /**
   * @brief Get the channel values object. The values are normalized to the range [-1, 1]. The provided indexes can help
   * identify which channel is which, e.g. kRC_THROTTLE_INDEX, kRC_ROLL_INDEX, etc.
   *
   * @return std::vector<float>
   */
  std::vector<float> get_channel_values();

  /**
   * @brief Check if the remote controller has received a single packet of data since being instantiated
   *
   * @return true if the remote controller has received a packet of data
   * @return false if the remote controller has not received a packet of data
   */
  bool has_received_data() const;

  /**
   * @brief If we lose connection with the remote controller, we likely want to stop the system. This function spawns a
   * thread that monitors the connection, and invokes the callback if the connection is lost.
   *
   * @param callback Callback function to invoke if the connection with the remote controller is lost. This should
   * likely shut down the system.
   */
  void packet_loss_callback(std::function<void()> callback);

  /**
   * @brief Shuts down the thread that monitors for packet loss. The callback will no longer be invoked if the
   * connection is lost.
   *
   */
  void stop_packet_loss_monitoring();

 private:
  /**
   * @brief Construct a new Remote Controller object. Private because this is a singleton.
   *
   */
  RemoteController();

  /**
   * @brief Destroy the Remote Controller object. Private because this is a singleton.
   *
   */
  ~RemoteController();

  /**
   * @brief Callback function for the rc_dsm module. This function is called whenever new data is available from the
   * remote controller.
   *
   */
  static void data_callback();

  /**
   * @brief Set the channel values object. This function is called by the data_callback function.
   *
   * @param values The values of the channel.
   */
  void set_channel_values(std::vector<float> &&values);

  /**
   * @brief Thread to make sure that the connection to the remote controller is not lost. If the connection is lost, the
   * program will exit via rc_set_state(EXITING), which all applications should monitor.
   *
   */
  void lost_connection_monitor();

  std::vector<float> rc_data_;             //< The data from the remote controller
  std::mutex rc_mutex_;                    //< Mutex to protect the rc_data_ variable
  std::atomic<bool> is_running_;           //< Flag to indicate if the connection monitor thread is running
  std::thread connection_monitor_thread_;  //< Thread to monitor the connection to the remote controller
  std::function<void()>
      packet_loss_callback_;  //< Callback function to invoke if the connection to the remote controller is lost
};

}  // namespace flyMS
