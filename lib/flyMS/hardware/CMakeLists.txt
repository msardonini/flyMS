
add_library(flyMS_hardware STATIC
  gps.cc
  Imu.cc
  RemoteController.cc
)

target_link_libraries(flyMS_hardware
  PUBLIC
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
    mavlink_v2
)

target_include_directories(flyMS_hardware
  PUBLIC
    ${FLYMS_LIBRARY_ROOT}
)

# Build with DEBUG defined if we are in Debug mode
if (CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(flyMS_hardware
    PUBLIC
      -DDEBUG_FLYMS
    )
endif()
