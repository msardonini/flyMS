
add_executable(flyMS_tests
  DigitalFilter_tests.cc
  ../DigitalFilter.cc
)

message(${PROJECT_SOURCE_DIR})

target_include_directories(flyMS_tests
  PUBLIC
    ${PROJECT_SOURCE_DIR}/lib
)

target_link_libraries(flyMS_tests
  PRIVATE
    gtest
    gtest_main
    flyMS
)
