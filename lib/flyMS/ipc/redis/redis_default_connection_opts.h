#pragma once

#include "redis++.h"

namespace flyMS {

static constexpr char kRedisHostDefault[] = "localhost";
static constexpr int kRedisPortDefault = 6379;
static constexpr std::chrono::duration<int64_t> kRedisTimeout = std::chrono::seconds(1);

/**
 * @brief Get the default Redis connection options. Defer to the environment variables REDIS_HOST and REDIS_PORT if
 * they are set, otherwise use the default values.
 *
 * @return sw::redis::ConnectionOptions
 */
inline sw::redis::ConnectionOptions redis_default_connection_opts() {
  sw::redis::ConnectionOptions opts;

  // Check the environment for the redis host and port
  auto redis_host = std::getenv("REDIS_HOST");
  if (redis_host) {
    opts.host = redis_host;
  } else {
    opts.host = kRedisHostDefault;
  }

  auto redis_port = std::getenv("REDIS_PORT");
  if (redis_port) {
    opts.port = std::stoi(redis_port);
  } else {
    opts.port = kRedisPortDefault;
  }

  opts.socket_timeout = kRedisTimeout;
  return opts;
}

}  // namespace flyMS
