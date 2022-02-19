
add_subdirectory(yaml-cpp)
add_subdirectory(librobotcontrol)
add_subdirectory(spdlog)
add_subdirectory(eigen)


add_library(HTTPRequest INTERFACE)

target_include_directories(HTTPRequest
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/HTTPRequest/include
)
