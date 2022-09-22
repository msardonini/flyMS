/**
 * @file RedisSubscriber.h
 * @author Mike Sardonini
 * @brief Declaration of RedisSubscriber class
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
 * @brief
 *
 */
class RedisSubscriber {
 public:
  /**
   * @brief Construct a new Redis Interface object
   *
   * @param max_queue_size The queue size of the Redis subscriber. If the queue is full, the subscriber will drop older
   * messages
   */
  RedisSubscriber(
      std::function<void(std::string, std::string)> callback,
      std::function<void(const sw::redis::Error &)> error_callback = RedisSubscriber::default_timeout_callback);

  /**
   * @brief Destroy the Redis Interface object
   *
   */
  ~RedisSubscriber();

  /**
   * @brief Subscribe to a Redis channel. This member function can be called multiple times to subscribe to multiple
   * channels
   *
   * @param channel The channel to subscribe to
   */
  void subscribe_to_channel(std::string_view channel);

  /**
   * @brief Set the timeout callback function. This function will be called when the Redis subscriber times out
   *
   * @param callback
   */
  void set_timeout_callback(std::function<void(const sw::redis::Error)> callback);

 private:
  /**
   * @brief Thread for the Redis subscriber. Runs in a loop until is_running_ is false
   *
   */
  void subscribe_thread();

  /**
   * @brief Default behavior for action when the Redis subscriber does not receive a message in the time frame specified
   *
   * @param err
   */
  static void default_timeout_callback(const sw::redis::Error &err);

  std::function<void(const sw::redis::Error &err)>
      on_timeout_;  //< Function to be called when the Redis subscriber times out

  std::atomic<bool> is_running_{false};  //< Flag to indicate if the subscriber thread is running
  std::thread subscribe_thread_;         //< Thread for the Redis subscriber

  std::unique_ptr<sw::redis::Redis> redis_;     //< Redis Interface object
  std::unique_ptr<sw::redis::Subscriber> sub_;  //< Redis subscriber object
};

}  // namespace flyMS
