/**
 * @file redis_interface.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Definition of RedisSubscriber class
 * @version 0.1
 * @date 2022-08-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "flyMS/redis/RedisSubscriber.h"

namespace flyMS {

RedisSubscriber::RedisSubscriber(std::function<void(std::string, std::string)> callback,
                                 std::function<void(const sw::redis::Error &)> error_callback)
    : on_timeout_(error_callback) {
  sw::redis::ConnectionOptions opts1;
  opts1.host = kRedisHost;
  opts1.port = kRedisPort;
  opts1.socket_timeout = kRedisTimeout;

  redis_ = std::make_unique<sw::redis::Redis>(opts1);
  sub_ = std::make_unique<sw::redis::Subscriber>(redis_->subscriber());
  sub_->on_message(callback);

  is_running_ = true;
  subscribe_thread_ = std::thread(&RedisSubscriber::subscribe_thread, this);
}

RedisSubscriber::~RedisSubscriber() {
  is_running_ = false;
  if (subscribe_thread_.joinable()) {
    subscribe_thread_.join();
  }
}

void RedisSubscriber::subscribe_to_channel(std::string_view channel) { sub_->subscribe(channel); }

void RedisSubscriber::set_timeout_callback(std::function<void(const sw::redis::Error)> callback) {
  on_timeout_ = callback;
}

void RedisSubscriber::default_timeout_callback(const sw::redis::Error &err) {
  spdlog::warn("Timeout exceeded, err: {}", err.what());
}

void RedisSubscriber::subscribe_thread() {
  while (is_running_.load()) {
    try {
      sub_->consume();
    } catch (const sw::redis::Error &err) {
      on_timeout_(err);
    }
  }
}

}  // namespace flyMS
