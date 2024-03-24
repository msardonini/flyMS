/**
 * @file ulog_messages.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Message definitions for ULog
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <array>
#include <cstdint>

#include "flyMS/common/state_data.h"

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
  virtual const char *get_msg_name() const = 0;
  virtual const char *get_msg_format() const = 0;
};

/**
 * @brief Data struct for storing metrics that are used to evaluate the performance of the flight controller. This
 * includes the system's state, sensor data, setpoints, and control effort (motor commands) This struct inherits from
 * ULogMsg and is used to write to the ULog file
 */
struct ULogFlightMsg : public ULogMsg {
  ULogFlightMsg() = default;

  ULogFlightMsg(uint64_t timestamp_us, const StateData &state, const std::vector<float> &setpoints,
                const std::vector<float> &u, const std::vector<float> &u_euler)
      : data(timestamp_us, state, setpoints, u, u_euler) {}

  /**
   * @brief The data fields that are recorded for the ULogFlightMsg. This struct is in serializable format
   *
   */
  struct ULogFlightMsgData {
    ULogFlightMsgData(uint64_t timestamp_us, const StateData &state, const std::vector<float> &setpoints,
                      const std::vector<float> &u, const std::vector<float> &u_euler)
        : timestamp_us(timestamp_us),
          RPY{state.euler(0), state.euler(1), state.euler(2)},
          gyro{state.gyro(0), state.gyro(1), state.gyro(2)},
          gyro_filt{state.eulerRate(0), state.eulerRate(1), state.eulerRate(2)},
          accel{state.accel(0), state.accel(1), state.accel(2)},
          motor_cmds{u[0], u[1], u[2], u[3]},
          u{u_euler[0], u_euler[1], u_euler[2], u_euler[3]},
          RPY_ref{setpoints[1], setpoints[2], setpoints[3]} {}

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
  static constexpr char NAME[] = "flight";                                          //< The name of the ULogFlightMsg
  static constexpr uint16_t ID = 0x0000;
  uint16_t get_msg_id() const override { return ID; }
  std::size_t get_msg_size() const override { return sizeof(data); }
  const void *get_msg_ptr() const override { return &data; }
  const char *get_msg_name() const override { return NAME; }
  const char *get_msg_format() const override { return FORMAT; }
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

  static constexpr char FORMAT[] = "gps:uint64_t timestamp;double[3] LLA;";  //< The format string for the ULogGpsMsg
  static constexpr char NAME[] = "gps";                                      //< The name of the ULogGpsMsg
  static constexpr uint16_t ID = 0x0001;                                     //< The ID of the ULogGpsMsg

  uint16_t get_msg_id() const override { return ID; }
  std::size_t get_msg_size() const override { return sizeof(data); }
  const void *get_msg_ptr() const override { return &data; }
  const char *get_msg_name() const override { return NAME; }
  const char *get_msg_format() const override { return FORMAT; }
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
  static constexpr char NAME[] = "pos_cntrl";      //< The name of the ULogPosCntrlMsg
  static constexpr uint16_t ID = 0x0002;           //< The ID of the ULogPosCntrlMsg

  // Required overrides
  uint16_t get_msg_id() const override { return ID; }
  std::size_t get_msg_size() const override { return sizeof(data); }
  const void *get_msg_ptr() const override { return &data; }
  const char *get_msg_name() const override { return NAME; }
  const char *get_msg_format() const override { return FORMAT; }
};

}  // namespace flyMS
