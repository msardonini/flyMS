/**
 * @file RedisPublisher.h
 * @author Mike Sardonini
 * @brief RedisPublisher class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <sstream>

#include "redis++.h"
#include "yaml-cpp/yaml.h"

namespace flyMS {

class RedisPublisher {
 public:
  RedisPublisher() {
    sw::redis::ConnectionOptions opts1;
    opts1.host = "127.0.0.1";
    opts1.port = 6379;
    opts1.socket_timeout = std::chrono::milliseconds(100);
    redis_ = std::make_unique<sw::redis::Redis>(opts1);
  }

  void publish(std::string_view channel, std::string_view msg) { redis_->publish(channel, msg); }

  void publish(std::string_view channel, const YAML::Node& msg) {
    std::stringstream msg_str;
    msg_str << msg;
    publish(channel, msg_str.str());
  }

 private:
  std::unique_ptr<sw::redis::Redis> redis_;
};

}  // namespace flyMS
