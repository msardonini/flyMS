
list(APPEND FLYMS_TEST_EXECUTABLES)

function(flyMS_add_unit_test unit_test_target)
  add_test(NAME ${unit_test_target}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin  
    COMMAND ./${unit_test_target}
  )

  target_code_coverage(${unit_test_target} ALL)

endfunction()