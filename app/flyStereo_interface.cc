/**
 * @file flyStereo_interface.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief This file contains the implementation of the interface between flyMS and flyStereo. It owns the serial port
 * that connects to the mission computer, and communicates with the flyMS app using redis. All messages are in mavlink
 * format, and mavlink is used to serialize, deserialize, messages
 * @version 0.1
 * @date 2022-07-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "flyMS/mavlink_interface.h"
#include "mavlink_v2/common/mavlink.h"
#include "redis++.h"

const std::string kMavlinkRedisChannel = "FlyStereoData";
std::atomic<bool> is_running_{true};

/**
 * @brief Signal handler. Ignore SIGHUP, others will shut down the process
 *
 * @param signo
 */
static void on_signal_received(int signo) {
  switch (signo) {
    case SIGHUP:
      break;
    default:
      is_running_ = false;
  }
}

int main() {
  // Register System Signals to be handled by on_signal_received
  signal(SIGINT, on_signal_received);
  signal(SIGKILL, on_signal_received);
  signal(SIGHUP, on_signal_received);

  sw::redis::ConnectionOptions opts1;
  opts1.host = "127.0.0.1";
  opts1.port = 6379;
  opts1.socket_timeout = std::chrono::milliseconds(100);

  auto redis = sw::redis::Redis(opts1);

  // Lambda function to forward mavlink messages to redis
  auto pub_mavlink_msg = [&redis](mavlink_message_t& msg) {
    uint8_t buf[1024];
    auto buf_size = mavlink_msg_to_send_buffer(buf, &msg);
    std::string buf_str(buf, buf + buf_size);
    redis.publish(kMavlinkRedisChannel, buf_str);
  };

  flyMS::MavlinkInterface mavlink_interface("/dev/ttyS5");

  mavlink_interface.register_callback({MAVLINK_MSG_ID_COMMAND_INT, pub_mavlink_msg});
  mavlink_interface.register_callback({MAVLINK_MSG_ID_HEARTBEAT, pub_mavlink_msg});

  // Odometry messages go to the flyMS application
  mavlink_interface.register_callback({MAVLINK_MSG_ID_ODOMETRY, pub_mavlink_msg});

  while (is_running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
