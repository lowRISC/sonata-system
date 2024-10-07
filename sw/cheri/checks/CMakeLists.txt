# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
set(CHECKS
  cheri_sanity.cc
  gpio_check.cc
  uart_check.cc
  spi_test.cc
  system_info_check.cc
  revocation_test.cc
  rgbled_test.cc
  usbdev_check.cc
)

foreach(CHECK ${CHECKS})
  get_filename_component(NAME ${CHECK} NAME_WE)

  add_executable(${NAME} ${CHECK})
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})
  target_link_libraries(${NAME} common)

  add_custom_command(
    TARGET ${NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
    COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
    VERBATIM
  )

  install(TARGETS ${NAME})
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
install(TARGETS ${NAME})

set(NAME hyperram_test)
add_executable(${NAME}  hyperram_test.cc)
target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})
target_link_libraries(${NAME} common)

add_custom_command(
  TARGET ${NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
  COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
  VERBATIM
)
install(TARGETS ${NAME})
