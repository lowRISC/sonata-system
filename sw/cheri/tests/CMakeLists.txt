set(TESTS
  cheri_sanity.cc
  gpio_check.cc
  uart_check.cc
  spi_test.cc
  revocation_test.cc
  rgbled_test.cc
)

foreach(TEST ${TESTS})
  get_filename_component(NAME ${TEST} NAME_WE)

  add_executable(${NAME} ${TEST} boot.S)
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})

  add_custom_command(
    TARGET ${NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
    COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
    VERBATIM
  )
endforeach()

set(NAME tag_test)

add_executable(${NAME} "tag_test.S")
target_include_directories(${NAME} PRIVATE "${CHERIOT_RTOS_SDK}/include")

add_custom_command(
  TARGET ${NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
  COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
  VERBATIM
)
