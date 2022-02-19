
cmake_minimum_required(VERSION 3.12)

project(FILTER)


add_library(filter
  STATIC
    filter.c
)

target_include_directories(filter
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS filter
	DESTINATION /usr/lib
)
