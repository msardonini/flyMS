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

/// \brief Queries the webserver for the flight configuration
///
/// \param uri The URI of the webserver to read from
/// \return A YAML::Node of the flight configuration
YAML::Node get_config(const std::string &uri);

}  // namespace flyMS
