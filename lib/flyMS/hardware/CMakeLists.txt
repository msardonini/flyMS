
add_library(flyMS_hardware STATIC
  gps.cc
  Imu.cc
  Setpoint.cc
)

target_link_libraries(flyMS_hardware
  PUBLIC
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
)

target_include_directories(flyMS_hardware
  PUBLIC
    ${FLYMS_LIBRARY_ROOT}
)
