/**
 * @file config_requestor.cc
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief Declaration of the Configuration Helper functions
 * @version 0.1
 * @date 2022-08-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <string>

#include "yaml-cpp/yaml.h"

namespace flyMS {

/**
 * @brief Get the flight configuration from the flyMS webserver and return it in the YAML::Node format
 *
 * @param uri The URI of the flyMS webserver
 * @return YAML::Node The flight configuration in the YAML::Node format
 */
YAML::Node get_config(const std::string &uri);

}  // namespace flyMS
