
add_executable(flyMS_controller_tests
  DigitalFilter_tests.cc
  AttitudeController_tests.cc
)

target_include_directories(flyMS_controller_tests
  PUBLIC
    ${PROJECT_SOURCE_DIR}/lib
)

target_link_libraries(flyMS_controller_tests
  PRIVATE
    gtest
    gtest_main
    flyMS_controller
)

flyMS_add_unit_test(flyMS_controller_tests)
