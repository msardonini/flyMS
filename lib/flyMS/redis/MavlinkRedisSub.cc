/**
 * @file redis_interface.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Source file for the MavlinkRedisSub, an object which assists in inter-process communication for using Redis
 * and Mavlink Messages
 * @version 0.1
 * @date 2022-08-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "flyMS/redis/MavlinkRedisSub.h"

namespace flyMS {

MavlinkRedisSub::MavlinkRedisSub(std::size_t max_queue_size)
    : RedisSubscriber(
          std::bind(&MavlinkRedisSub::MavlinkMessageCallback, this, std::placeholders::_1, std::placeholders::_2)),
      max_queue_size_(max_queue_size) {}

MavlinkRedisSub::~MavlinkRedisSub() {}

std::tuple<VioData, bool> MavlinkRedisSub::GetVioData() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (queue_vio_data_.empty()) {
    return std::make_tuple(VioData(), false);
  } else {
    auto data = queue_vio_data_.front();
    queue_vio_data_.pop();
    return std::make_tuple(data, true);
  }
}

VioData MavlinkRedisSub::ConvertFromMavlink(mavlink_odometry_t &msg) {
  VioData vio_data;
  vio_data.position << msg.x, msg.y, msg.z;
  vio_data.velocity << msg.vx, msg.vy, msg.vz;
  vio_data.quat = Eigen::Quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  return vio_data;
}

void MavlinkRedisSub::MavlinkMessageCallback(std::string channel, std::string msg) {
  for (auto mav_char : msg) {
    mavlink_message_t mav_msg;
    mavlink_status_t mav_status;
    auto found = mavlink_parse_char(MAVLINK_COMM_2, mav_char, &mav_msg, &mav_status);
    if (found) {
      switch (mav_msg.msgid) {
        case MAVLINK_MSG_ID_ODOMETRY: {
          mavlink_odometry_t odometry_msg;
          mavlink_msg_odometry_decode(&mav_msg, &odometry_msg);
          auto vio_data = ConvertFromMavlink(odometry_msg);
          std::lock_guard<std::mutex> lock(queue_mutex_);
          queue_vio_data_.push(vio_data);
          if (queue_vio_data_.size() > max_queue_size_) {
            queue_vio_data_.pop();
          }
          break;
        }
      }
    }
  }
}

}  // namespace flyMS
