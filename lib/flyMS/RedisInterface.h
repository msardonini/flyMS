/**
 * @file redis_interface.h
 * @author Mike Sardonini
 * @brief Header file for the RedisInterface, an object which assists in inter-process communication for using Redis and
 * Mavlink Messages
 *
 * @version 0.1
 * @date 2022-08-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "Eigen/Dense"
#include "flyMS/types/vio_data.h"
#include "mavlink_v2/common/mavlink.h"
#include "redis++.h"
#include "spdlog/spdlog.h"

namespace flyMS {

constexpr char kRedisHost[] = "127.0.0.1";
constexpr int kRedisPort = 6379;
constexpr std::chrono::duration<int64_t> kRedisTimeout = std::chrono::seconds(1);

/**
 * @brief Object to interface with Redis for flyMS. Mavlink encoded messages are received with a Redis subscriber,
 * which are then parsed and given to the user. Also, Mavlink messages can be sent with a Redis publisher.
 *
 */
class RedisInterface {
 public:
  /**
   * @brief Construct a new Redis Interface object
   *
   * @param max_queue_size The queue size of the Redis subscriber. If the queue is full, the subscriber will drop older
   * messages
   */
  RedisInterface(std::size_t max_queue_size);

  /**
   * @brief Destroy the Redis Interface object
   *
   */
  ~RedisInterface();

  /**
   * @brief Subscribe to a Redis channel
   *
   * @param channel The channel to subscribe to
   */
  void subscribe_to_channel(std::string_view channel);

  /**
   * @brief Gets the latest VioData message from the queue. If the queue is empty, it will return empty
   *
   * @return std::tuple<VioData, bool> Tuple of the latest VioData message and a bool indicating if the message was
   * filled
   */
  std::tuple<VioData, bool> GetVioData();

  // TODO implement this
  // void SendStartCommand() {}
  // void SendStopCommand() {}

 private:
  /**
   * @brief Converts a mavlink_odometry_t messages to a VioData object
   *
   * @param msg The mavlink_odometry_t message to convert
   * @return VioData The converted VioData object
   */
  VioData ConvertFromMavlink(mavlink_odometry_t &msg);
  /**
   * @brief Callback for the Redis subscriber. Parses the message and adds it to the queue. If the queue is full, older
   * messages are dropped
   *
   * @param channel The channel the message was received on
   * @param msg The contents of the message
   */
  void MavlinkMessageCallback(std::string channel, std::string msg);

  /**
   * @brief Thread for the Redis subscriber. Runs in a loop until is_running_ is false
   *
   */
  void subscribe_thread();

  std::atomic<bool> is_running_{false};  //< Flag to indicate if the subscriber thread is running
  std::thread subscribe_thread_;         //< Thread for the Redis subscriber
  std::mutex queue_mutex_;               //< Mutex for the queue
  std::size_t max_queue_size_;           //< The maximum queue size of the Redis subscriber
  std::queue<VioData> queue_vio_data_;   //< Queue for the VioData messages

  std::unique_ptr<sw::redis::Redis> redis_;     //< Redis Interface object
  std::unique_ptr<sw::redis::Subscriber> sub_;  //< Redis subscriber object
};

}  // namespace flyMS
