

add_library(flyMS_ipc STATIC
  redis/RedisSubscriber.cc
  mavlink/MavlinkRedisSub.cc
  mavlink/MavlinkUart.cc
  uart/Uart.cc
)

target_link_libraries(flyMS_ipc
  PUBLIC
    spdlog
    robotics_cape
    redis++_static
    mavlink_v2
    yaml-cpp
)

target_include_directories(flyMS_ipc
  PUBLIC
    ${FLYMS_LIBRARY_ROOT}
)

if (BUILD_TESTING)
  add_subdirectory(tests)
endif()
