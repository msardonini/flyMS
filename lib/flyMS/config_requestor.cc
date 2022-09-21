/**
 * @file config_requestor.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Implementation of the Configuration Helper functions
 * @version 0.1
 * @date 2022-08-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "flyMS/config_requestor.h"

#include "HTTPRequest.hpp"
#include "spdlog/spdlog.h"

namespace flyMS {

/**
 * @brief Get the flight configuration from the flyMS webserver and return it in the YAML::Node format
 *
 * @param uri The URI of the flyMS webserver
 * @return YAML::Node The flight configuration in the YAML::Node format
 */
YAML::Node get_config(const std::string &uri) {
  try {
    // you can pass http::InternetProtocol::V6 to Request to make an IPv6
    // request
    http::Request request{uri};

    // send a get request
    const auto response = request.send("GET", "", std::vector<std::string>{}, std::chrono::seconds(3));
    auto response_str = std::string{response.body.begin(), response.body.end()};
    auto node = YAML::Load(response_str);
    return node;
  } catch (const std::exception &e) {
    spdlog::error("HTTP Request failed on URI {}, error: {}", uri, e.what());
    return YAML::Node();
  }
}

}  // namespace flyMS
