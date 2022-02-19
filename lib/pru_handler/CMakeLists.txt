cmake_minimum_required(VERSION 3.5)

add_library(pru_handler STATIC
  pru_handler.cc
)

target_link_libraries(pru_handler
  PUBLIC
    robotics_cape
    pthread
    Eigen3::Eigen
)

target_include_directories(pru_handler
  PUBLIC
    ${LIBRARY_ROOT}
)

install(TARGETS pru_handler
  DESTINATION /usr/bin/
)
