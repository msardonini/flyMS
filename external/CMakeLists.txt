
add_subdirectory(yaml-cpp)
add_subdirectory(spdlog)
add_subdirectory(eigen)

cmake_policy(SET CMP0079 NEW)
add_subdirectory(librobotcontrol)
target_include_directories(robotics_cape
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/librobotcontrol/library/include
)
target_link_libraries(robotics_cape
  PRIVATE
    m
    rt
    pthread
)

add_library(HTTPRequest INTERFACE)
target_include_directories(HTTPRequest
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/HTTPRequest/include
)
