
#include <string>

#include "yaml-cpp/yaml.h"

namespace flyMS {

/// \brief Queries the webserver for the flight configuration
///
/// \param uri The URI of the webserver to read from
/// \return A YAML::Node of the flight configuration
YAML::Node get_config(const std::string &uri);

} // namespace flyMS
