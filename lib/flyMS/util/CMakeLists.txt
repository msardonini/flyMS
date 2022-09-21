


add_library(flyMS_util STATIC
  ready_check.cc
  config_requestor.cc
)

target_link_libraries(flyMS_util
PUBLIC
  spdlog
  robotics_cape
  HTTPRequest
  yaml-cpp
)

target_include_directories(flyMS_util
PUBLIC
  ${FLYMS_LIBRARY_ROOT}
)
