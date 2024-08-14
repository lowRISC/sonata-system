set(TESTS
  test_runner.cc
)

foreach(TEST ${TESTS})
  get_filename_component(NAME ${TEST} NAME_WE)

  add_executable(${NAME} ${TEST} boot.S)
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})

endforeach()
