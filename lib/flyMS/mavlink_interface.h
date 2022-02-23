#pragma once

#include <atomic>
#include <thread>

#include "flyMS/types/state_data.h"
#include "flyMS/types/vio_data.h"
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
   * @param input_params Input configuration with a YAML::Node
   */
  MavlinkInterface(const YAML::Node input_params);
  ~MavlinkInterface();

  int Init();

  operator bool() { return is_running_.load(); }

  /**
   * @brief Sends the IMU message over the mavlink serial bus
   *
   * @param imu_state The data to send
   * @return int 0 on success, -1 on failure
   */
  int SendImuMessage(const StateData &imu_state);

  /**
   * @brief Sends the start command, which signals to start measuring VIO in flyStereo
   *
   * @return int 0 on success, -1 on failure
   */
  int SendStartCommand();

  /**
   * @brief Sends the shutdown command, which signals flyStereo to stop measuring
   *
   * @return int 0 on success, -1 on failure
   */
  int SendShutdownCommand();

  /**
   * @brief Get the Visual Inertial Odometry (Vio) Data object
   *
   * @param vio the pointer to insert data to
   * @return true if new data has been received and vio is populated, false otherwise
   */
  bool GetVioData(VioData *vio);

 private:
  /**
   * @brief Thread the reads mavlink serial data
   *
   */
  void SerialReadThread();

  /**
   * @brief Thread which monitors the GPIO pin that the camera triggers on image capture
   *
   */
  void GpioThread();

  /**
   * @brief Resets the trigger count
   *
   */
  void ResetCounter();

  std::atomic<bool> is_running_;    //< Flag to tell if we are still operating
  int serial_dev_;                  //< file descriptor for the serial port
  std::string serial_dev_file_;     //< Filename of the serial port in the device tree
  std::thread serial_read_thread_;  //< Thread that reads the serial port for new data
  std::mutex serial_send_mutex_;    //< Mutex which prevents us from writing to the serial port at the same time
  std::mutex vio_mutex_;            //< mutex for the vio data object
  VioData vio_;                     //< contains data from the visual inertial odometry system (flyStereo)
  bool is_new_vio_data_ = false;    //< Flag if via data is new, i.e. hasn't been read yet

  // Variables for handling the GPIO line which is the camera trigger
  std::thread gpio_thread_;        //< Thread which monitors the gpio pin which triggers on image capture
  uint64_t trigger_time_;          //< The recorded time of the pulse
  uint32_t trigger_count_;         //< The counter of the image
  std::mutex trigger_time_mutex_;  //< The trigger time
};

}  // namespace flyMS
