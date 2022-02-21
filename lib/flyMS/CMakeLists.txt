cmake_minimum_required(VERSION 3.5)

project(flyMS VERSION 1.0.0)


add_definitions(-DDEBUG)


#Build flyMS
add_library(flyMS STATIC
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
    filter
    yaml-cpp
    HTTPRequest
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
