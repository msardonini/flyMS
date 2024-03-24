

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

# Build with DEBUG defined if we are in Debug mode
if (CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(flyMS_ipc
    PUBLIC
      -DDEBUG_FLYMS
    )
endif()

if (FLYMS_BUILD_TESTS)
  add_subdirectory(test)
endif()
