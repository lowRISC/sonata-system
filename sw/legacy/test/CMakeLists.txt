# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_executable(memory_test memory_test.c)
target_link_libraries(memory_test common)

install(TARGETS memory_test)

add_custom_command(
  TARGET memory_test POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:memory_test>" "$<TARGET_FILE:memory_test>.bin"
  COMMAND srec_cat "$<TARGET_FILE:memory_test>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:memory_test>.vmem" -vmem
  VERBATIM)

add_executable(spi_test spi_test.c)
target_link_libraries(spi_test common)

install(TARGETS spi_test)

add_custom_command(
  TARGET spi_test POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:spi_test>" "$<TARGET_FILE:spi_test>.bin"
  COMMAND srec_cat "$<TARGET_FILE:spi_test>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:spi_test>.vmem" -vmem
  VERBATIM)

add_executable(heap_test heap_test.c)
target_link_libraries(heap_test common syscall)
target_link_options(heap_test PRIVATE "-specs=nosys.specs")

add_custom_command(
  TARGET heap_test POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:heap_test>" "$<TARGET_FILE:heap_test>.bin"
  COMMAND srec_cat "$<TARGET_FILE:heap_test>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:heap_test>.vmem" -vmem
  VERBATIM)
