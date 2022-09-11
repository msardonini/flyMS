/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "flyMS/setpoint.h"
#include "flyMS/types/state_data.h"
#include "flyMS/types/vio_data.h"

namespace flyMS {

/**
 * @brief Base class for all ULog data structs. This sets the interface for how ULog data is processed to store on disk
 *
 */
struct ULogMsg {
  virtual ~ULogMsg() = default;
  virtual uint16_t get_msg_id() const = 0;
  virtual std::size_t get_msg_size() const = 0;
  virtual const void *get_msg_ptr() const = 0;
};

/**
 * @brief Data struct for storing metrics that are used to evaluate the performance of the flight controller. This
 * includes the system's state, sensor data, setpoints, and control effort (motor commands) This struct inherits from
 * ULogMsg and is used to write to the ULog file
 */
struct ULogFlightMsg : public ULogMsg {
  ULogFlightMsg() = default;

  ULogFlightMsg(uint64_t timestamp_us, const StateData &state, const SetpointData &setpoint,
                const std::array<float, 4> &u, const std::array<float, 3> u_euler)
      : data(timestamp_us, state, setpoint, u, u_euler) {}

  /**
   * @brief The data fields that are recorded for the ULogFlightMsg. This struct is in serializable format
   *
   */
  struct ULogFlightMsgData {
    ULogFlightMsgData(uint64_t timestamp_us, const StateData &state, const SetpointData &setpoint,
                      const std::array<float, 4> &u, const std::array<float, 3> u_euler)
        : timestamp_us(timestamp_us),
          RPY{state.euler(0), state.euler(1), state.euler(2)},
          gyro{state.gyro(0), state.gyro(1), state.gyro(2)},
          gyro_filt{state.eulerRate(0), state.eulerRate(1), state.eulerRate(2)},
          accel{state.accel(0), state.accel(1), state.accel(2)},
          motor_cmds{u[0], u[1], u[2], u[3]},
          u{u_euler[0], u_euler[1], u_euler[2], setpoint.throttle},
          RPY_ref{setpoint.euler_ref[0], setpoint.euler_ref[1], setpoint.euler_ref[2]} {}

    uint64_t timestamp_us;  //< Timestamp in microseconds
    float RPY[3];           //< Roll, pitch, yaw in radians
    float gyro[3];          //< Gyro measurements in rad/s
    float gyro_filt[3];     //< Filtered gyro measurements in rad/s
    float accel[3];         //< Accelerometer measurements in m/s^2
    float motor_cmds[4];    //< Motor commands in [0, 1]
    float u[4];             //< Control effort (throttle, roll, pitch, yaw) in [0, 1]
    float RPY_ref[3];       //< Reference roll, pitch, yaw in radians
  } __attribute__((packed));

  ULogFlightMsgData data;  //< The data fields that are recorded for the ULogFlightMsg

  static constexpr char FORMAT[] =
      "flight:uint64_t timestamp;float[3] RPY;float[3] gyro;float[3] "
      "gyro_filt;float[3] accel;float[4] motor_cmds;float[4] u;float[3] RPY_ref;";  //< The format string for
                                                                                    // the ULogFlightMsg
  static constexpr uint16_t ID = 0x0000;
  virtual uint16_t get_msg_id() const override { return ID; }
  virtual std::size_t get_msg_size() const override { return sizeof(data); }
  virtual const void *get_msg_ptr() const override { return &data; }
};

/**
 * @brief Data struct for storing GPS data. This struct inherits from ULogMsg and is used to write to the ULog file.
 * It contains the recorded GPS LLA data
 *
 */
struct ULogGpsMsg : public ULogMsg {
  // Default Constructor
  ULogGpsMsg() = default;

  // Data Constructor
  ULogGpsMsg(uint64_t timestamp_us, double LLA[3]) : data(timestamp_us, LLA) {}

  struct ULogGpsMsgData {
    ULogGpsMsgData(uint64_t _timestamp_us, double LLA[3]) : timestamp_us(_timestamp_us), LLA{LLA[0], LLA[1], LLA[2]} {}

    uint64_t timestamp_us;
    double LLA[3];
  } __attribute__((packed));

  ULogGpsMsgData data;

  static constexpr char FORMAT[] = "gps:uint64_t timestamp;double[3] LLA;";
  static constexpr uint16_t ID = 0x0001;

  virtual uint16_t get_msg_id() const override { return ID; }
  virtual std::size_t get_msg_size() const override { return sizeof(data); }
  virtual const void *get_msg_ptr() const override { return &data; }
};

/**
 * @brief Data struct for storing the position and velocity of the vehicle. This struct inherits from ULogMsg and is
 * used to write to the ULog file. This struct is only used when the flyMS system is receiving position and velocity
 * data from an external source (e.g. VIO). It records here to show it's consistency with the rest of the flyMS data
 */
struct ULogPosCntrlMsg : public ULogMsg {
  ULogPosCntrlMsg(uint64_t _timestamp_us, float _position[3], float velocity[3], float quat[4], float command_RPWT[4])
      : data(_timestamp_us, _position, velocity, quat, command_RPWT) {}

  /**
   * @brief The data fields that are recorded for the ULogPosCntrlMsg. This struct is in serializable format
   *
   */
  struct ULogPosCntrlMsgData {
    ULogPosCntrlMsgData(uint64_t timestamp_us, float position[3], float velocity[3], float quat[4],
                        float command_RPWT[4])
        : timestamp_us(timestamp_us),
          position{position[0], position[1], position[2]},
          velocity{velocity[0], velocity[1], velocity[2]},
          quat{quat[0], quat[1], quat[2], quat[3]},
          command_RPWT{command_RPWT[0], command_RPWT[1], command_RPWT[2], command_RPWT[3]} {}

    uint64_t timestamp_us;  //< Timestamp in microseconds
    float position[3];      //< Position in meters
    float velocity[3];      //< Velocity in m/s
    float quat[4];          //< Quaternion
    float command_RPWT[4];  //< Commanded roll, pitch, yaw, throttle
  } __attribute__((packed));

  ULogPosCntrlMsgData data;  //< The data fields that are recorded for the ULogPosCntrlMsg

  static constexpr char FORMAT[] =
      "pos_cntrl:uint64_t timestamp;float[3] position;float[3] "
      "velocity;float[4] quat;float[4] command;";  //< The format string for the ULogPosCntrlMsg
  static constexpr uint16_t ID = 0x0002;           //< The ID of the ULogPosCntrlMsg

  // Required overrides
  virtual uint16_t get_msg_id() const override { return ID; }
  virtual std::size_t get_msg_size() const override { return sizeof(data); }
  virtual const void *get_msg_ptr() const override { return &data; }
};

/**
 * @brief Class to handle the logging of data to a file in the ULog data format
 * (https://docs.px4.io/main/en/dev_log/ulog_file_format.html). The interface is relatively simple. The user
 * instantiates the object, runs the init() member function once, and then repeatedly calls the `write_msg()`
 * member function to record the data to a file. The user can then use the `px4tools` python package to read the data
 * from the file (typically it is converted to a CSV file). To increase the performance of the logging, the interaction
 * with the file system is done in a separate thread. This way, the control thread doesn't get backed up with I/O
 * flushing
 *
 */
class ULog {
 public:
  /**
   * @brief Construct a new ULog object
   *
   */
  ULog() = default;

  /**
   * @brief Destroy the ULog object
   *
   */
  ~ULog();

  /**
   * @brief Initialize the ULog object. This function must be called before any other member functions. It creates the
   * file, writes the header, and starts the logging thread. The created file is called logger.ulog, and it placed in
   * the directory provided
   *
   * @param log_folder The directory to save the log in
   * @return int 0 if successful, -1 otherwise
   */
  int init(const std::string &log_folder);

  /**
   * @brief Writes a generic ULogMsg to the log file. It serializes the data, then adds it to a queue to be written to
   * the file. The queue is then flushed to the file in the logging thread
   *
   * @param data The ULogMsg to write to the file
   */
  void write_msg(const ULogMsg &data);

 private:
  /**
   * @brief Get the time in microseconds since the epoch
   *
   * @return uint64_t
   */
  uint64_t get_time_us();

  /**
   * @brief Writes the generic ULog header to the file
   *
   */
  void write_header();

  /**
   * @brief Writes the ULog msg format to the header portion of the file
   *
   * @param msg_format The message format to write in the header
   */
  void write_formats(const std::string &msg_format);

  /**
   * @brief Registers the Ulog message formats with a name and ID
   *
   * @param id The ID of the message
   * @param msg_name The name of the message
   */
  void write_add_log(uint16_t id, std::string msg_name);

  /**
   * @brief Write bytes to the file
   *
   * @param buf Pointer to the data to write
   * @param size The number of bytes to write
   */
  void write_to_file(const void *buf, size_t size);

  /**
   * @brief The thread that handles the flushing of the data to the file
   *
   */
  void write_thread();

  std::atomic<bool> is_running_ = false;                                           //< thread running flag
  std::thread write_thread_;                                                       //< thread for writing data
  std::mutex queue_mutex_;                                                         //< mutex for accessing the queue
  std::queue<std::pair<std::unique_ptr<const char[]>, std::size_t>> write_queue_;  //< queue for writing data
  std::ofstream fd_;  //< File descriptor for interfacing with the file
};

}  // namespace flyMS
