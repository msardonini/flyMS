cmake_minimum_required(VERSION 3.5)

add_definitions(-DDEBUG)


#Build flyMS
add_library(flyMS STATIC
  DigitalFilter.cc
  FlightCore.cc
  config_requestor.cc
  gps.cc
  imu/Imu.cc
  pru_client.cc
  setpoint.cc
  position_controller.cc
  mavlink_interface.cc
  ready_check.cc
  ulog/ulog.cc
)

target_link_libraries(flyMS
  PUBLIC
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
    HTTPRequest
    mavlink_v2
    redis++_static
)

target_include_directories(flyMS
  PUBLIC
    ${LIBRARY_ROOT}
)

target_compile_options(flyMS
  PUBLIC
    -Wall
    -Wno-address-of-packed-member
)

install(TARGETS flyMS
  DESTINATION /usr/bin/)


add_subdirectory(tests)
