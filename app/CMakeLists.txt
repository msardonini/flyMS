
# Build test webserver program
add_executable(test_webserver
  test_webserver.cc
)
target_link_libraries(test_webserver
  HTTPRequest
  yaml-cpp
)

# Build pru_handler app
add_executable(pru_handler_app
  pru_handler_app.cc
)
target_link_libraries(pru_handler_app
  PRIVATE
    pru_handler
)

# Build flyMS_app
add_executable(flyMS_app
  flyMS_app.cc
)
target_link_libraries(flyMS_app
  PRIVATE
    flyMS
)
set_target_properties(flyMS_app test_webserver pru_handler_app
  PROPERTIES
  LINK_FLAGS "-static-libstdc++ -static-libgcc"
  # LINK_FLAGS "-static"
)
