
add_executable(flyMS_ipc_tests
  redis_tests.cc
  mavlink_redis_tests.cc
)

target_link_libraries(flyMS_ipc_tests
  PRIVATE
    flyMS_ipc
    gtest
    gtest_main
)

flyMS_add_unit_test(flyMS_ipc_tests)
