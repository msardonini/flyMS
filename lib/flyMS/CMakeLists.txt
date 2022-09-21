cmake_minimum_required(VERSION 3.5)

# Build with DEBUG defined if we are in Debug mode
if (CMAKE_BUILD_TYPE MATCHES Debug)
  add_definitions(-DDEBUG)
endif()

add_subdirectory(hardware)
add_subdirectory(util)

#Build flyMS_lib
add_library(flyMS_lib STATIC
  redis/RedisSubscriber.cc
  redis/MavlinkRedisSub.cc
  DigitalFilter.cc
  FlightCore.cc
  position_controller.cc
  mavlink_interface.cc
  ulog/ulog.cc
)

target_link_libraries(flyMS_lib
  PUBLIC
    flyMS_util
    flyMS_hardware
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
    mavlink_v2
    redis++_static
    stdc++fs
)

target_include_directories(flyMS_lib
  PUBLIC
    ${FLYMS_LIBRARY_ROOT}
)

target_compile_options(flyMS_lib
  PUBLIC
    -Wall
    -Wno-address-of-packed-member
)

install(TARGETS flyMS_lib flyMS_util flyMS_hardware
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)


if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
