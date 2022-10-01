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

#include "flyMS/ipc/redis/RedisSubscriber.h"

#include "flyMS/ipc/redis/redis_default_connection_opts.h"

namespace flyMS {

RedisSubscriber::RedisSubscriber(std::function<void(std::string, std::string)> callback,
                                 const std::vector<std::string> &channels,
                                 std::function<void(const sw::redis::Error &)> error_callback)
    : on_error_(error_callback) {
  auto opts = redis_default_connection_opts();
  redis_ = std::make_unique<sw::redis::Redis>(opts);
  sub_ = std::make_unique<sw::redis::Subscriber>(redis_->subscriber());

  for (const auto &channel : channels) {
    sub_->subscribe(channel);
  }
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

void RedisSubscriber::set_error_callback(std::function<void(const sw::redis::Error)> callback) { on_error_ = callback; }

// Default behavior is to do nothing
void RedisSubscriber::default_timeout_callback(const sw::redis::Error &err) {}

void RedisSubscriber::subscribe_thread() {
  while (is_running_.load()) {
    try {
      sub_->consume();
    } catch (const sw::redis::Error &err) {
      on_error_(err);
    }
  }
}

}  // namespace flyMS
