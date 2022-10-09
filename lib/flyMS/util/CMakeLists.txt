
add_library(flyMS_util STATIC
  ready_check.cc
  config_requestor.cc
  pid_file.cc
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

# Build with DEBUG defined if we are in Debug mode
if (CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(flyMS_util
    PUBLIC
      -DDEBUG_FLYMS
    )
endif()
