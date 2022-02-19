
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
    const auto response = request.send("GET");
    auto response_str = std::string{response.body.begin(), response.body.end()};
    std::cout << response_str << std::endl;
    return YAML::Load(response_str);
  } catch (const std::exception &e) {
    std::cerr << "Request failed, error: " << e.what() << '\n';
    return YAML::Node();
  }
}

}  // namespace flyMS
