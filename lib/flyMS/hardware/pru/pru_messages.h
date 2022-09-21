/**
 * @file pru_messages.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Message definitions for the PRU. These are used by the PruRequester and PruManager to communicate with each
 * other. They are convertible to YAML::Node format and back, which is used for inter-process communication.
 * @version 0.1
 * @date 2022-09-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "flyMS/util/yaml_serialization.h"

namespace flyMS {

static constexpr char kredis_request_channel[] = "pru_request";
static constexpr char kredis_release_channel[] = "pru_release";
static constexpr char kredis_request_response_channel[] = "pru_request_response";
static constexpr char kredis_release_response_channel[] = "pru_release_response";

struct PruRequest {
  PruRequest() = default;
  int pid;

  bool operator==(const PruRequest& rhs) const { return std::tie(pid) == std::tie(rhs.pid); }

  constexpr static auto properties = std::make_tuple(property(&PruRequest::pid, "pid"));

  YAML::Node yaml() const { return to_yaml(*this); }
  // PruRequest(const YAML::Node& node) { from_yaml<PruRequest>(node); }
};

struct PruResponse {
  PruResponse() = default;
  bool success;
  std::string reason;

  bool operator==(const PruResponse& rhs) const {
    return std::tie(success, reason) == std::tie(rhs.success, rhs.reason);
  }

  constexpr static auto properties =
      std::make_tuple(property(&PruResponse::success, "success"), property(&PruResponse::reason, "reason"));

  YAML::Node yaml() const { return to_yaml(*this); }
  // PruResponse(const YAML::Node& node) { from_yaml<PruResponse>(node); }
};

}  // namespace flyMS
