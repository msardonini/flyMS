

add_executable(flyMS_ipc_tests
  redis_tests.cc
  mavlink_redis_tests.cc
)

target_link_libraries(flyMS_ipc_tests
  flyMS_ipc
  gtest
  gtest_main
)