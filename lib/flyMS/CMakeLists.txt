cmake_minimum_required(VERSION 3.5)

add_definitions(-DDEBUG)


#Build flyMS_lib
add_library(flyMS_lib STATIC
  redis/RedisSubscriber.cc
  redis/MavlinkRedisSub.cc
  DigitalFilter.cc
  FlightCore.cc
  config_requestor.cc
  gps.cc
  imu/Imu.cc
  setpoint.cc
  position_controller.cc
  mavlink_interface.cc
  ready_check.cc
  ulog/ulog.cc
)

target_link_libraries(flyMS_lib
  PUBLIC
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
    HTTPRequest
    mavlink_v2
    redis++_static
    stdc++fs
)

target_include_directories(flyMS_lib
  PUBLIC
    ${LIBRARY_ROOT}
)

target_compile_options(flyMS_lib
  PUBLIC
    -Wall
    -Wno-address-of-packed-member
)

install(TARGETS flyMS_lib
  DESTINATION /usr/bin/)


if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
