
#include "flyMS/config_requestor.h"

#include <iostream>

#include "HTTPRequest.hpp"

namespace flyMS {

YAML::Node get_config(const std::string &uri) {
  try {
    // you can pass http::InternetProtocol::V6 to Request to make an IPv6
    // request
    http::Request request{uri};

    // send a get request
    const auto response = request.send("GET", "", std::vector<std::string>{}, std::chrono::seconds(3));
    auto response_str = std::string{response.body.begin(), response.body.end()};
    auto node = YAML::Load(response_str)["data"];
    std::cout << node << std::endl;
    return node;
  } catch (const std::exception &e) {
    std::cerr << "Request failed, error: " << e.what() << '\n';
    return YAML::Node();
  }
}

}  // namespace flyMS
