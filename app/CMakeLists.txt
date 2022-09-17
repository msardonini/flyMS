
# Build test webserver program
add_executable(test_webserver
  test_webserver.cc
)
target_link_libraries(test_webserver
  HTTPRequest
  yaml-cpp
)

# Build PruManager
add_executable(PruManager
  PruManager.cc
)
target_link_libraries(PruManager
  PRIVATE
    flyMS_lib
    redis++_static
)

# Build flyMS
add_executable(flyMS
  flyMS.cc
)
target_link_libraries(flyMS
  PRIVATE
    flyMS_lib
)

add_executable(flyStereo_interface
  flyStereo_interface.cc
)
target_link_libraries(flyStereo_interface
  PRIVATE
    flyMS_lib
    redis++_static
)

set_target_properties(flyMS test_webserver PruManager flyStereo_interface
  PROPERTIES
  LINK_FLAGS "-static-libstdc++ -static-libgcc"
  # LINK_FLAGS "-static"
)
