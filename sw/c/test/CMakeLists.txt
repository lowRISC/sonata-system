add_executable(memory_test memory_test.c)
target_link_libraries(memory_test common)

add_custom_command(
  TARGET memory_test POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:memory_test>" "$<TARGET_FILE:memory_test>.bin"
  COMMAND srec_cat "$<TARGET_FILE:memory_test>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:memory_test>.vmem" -vmem
  VERBATIM)
