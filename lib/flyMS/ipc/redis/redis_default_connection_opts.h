#pragma once

#include "redis++.h"

namespace flyMS {

static constexpr char kRedisHost[] = "localhost";
static constexpr int kRedisPort = 6379;
static constexpr std::chrono::duration<int64_t> kRedisTimeout = std::chrono::seconds(1);

/**
 * @brief Get the default Redis connection options
 *
 * @return sw::redis::ConnectionOptions
 */
inline sw::redis::ConnectionOptions redis_default_connection_opts() {
  sw::redis::ConnectionOptions opts;
  opts.host = kRedisHost;
  opts.port = kRedisPort;
  opts.socket_timeout = kRedisTimeout;
  return opts;
}

}  // namespace flyMS
