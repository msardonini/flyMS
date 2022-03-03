cmake_policy(SET CMP0048 NEW)
project(flyMS LANGUAGES CXX C VERSION 0.1.0)
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

# Set c++ standard
set(CMAKE_CXX_STANDARD 20)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -Wall") # Uncomment to profile with gprof
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g")

# Set the binary and library directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(external)

# Build the Digital Filter Library
add_subdirectory(lib)

add_subdirectory(app)
