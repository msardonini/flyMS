cmake_minimum_required(VERSION 3.5)

add_subdirectory(hardware)
add_subdirectory(util)
add_subdirectory(ipc)

# Build flyMS_lib
add_library(flyMS_lib STATIC
  DigitalFilter.cc
  FlightCore.cc
  position_controller.cc
  ulog/ulog.cc
)

target_link_libraries(flyMS_lib
  PUBLIC
    flyMS_util
    flyMS_hardware
    flyMS_ipc
    spdlog
    robotics_cape
    Eigen3::Eigen
    yaml-cpp
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

# Build with DEBUG defined if we are in Debug mode
if (CMAKE_BUILD_TYPE MATCHES Debug)
  target_compile_definitions(flyMS_lib
    PUBLIC
      -DDEBUG_FLYMS
    )
endif()


if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
