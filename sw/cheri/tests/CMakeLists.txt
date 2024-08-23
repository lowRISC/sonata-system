set(TESTS
  test_runner.cc
)

foreach(TEST ${TESTS})
  get_filename_component(NAME ${TEST} NAME_WE)

  add_executable(${NAME} ${TEST})
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})
  target_link_libraries(${NAME} common)

  install(TARGETS ${NAME})
endforeach()
