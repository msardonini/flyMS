cmake_minimum_required(VERSION 3.5)

# # Get the architecture of the CPU building this
EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
string(COMPARE EQUAL "${ARCHITECTURE}" x86_64 host64bit)

if(host64bit)
  # Set the location of the toolchain file if needed
  set(CMAKE_TOOLCHAIN_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain.cmake"
    CACHE FILEPATH
    "toolchain path" FORCE)

  message(STATUS "Cross compiling to ARM   ${CMAKE_TOOLCHAIN_FILE}")
endif(host64bit)



project(flyMS LANGUAGES CXX C)

# Set c++ standard
set (CMAKE_CXX_STANDARD 20)

# Set the binary and library directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(external)

# Build the Digital Filter Library
add_subdirectory(lib)

add_subdirectory(app)
