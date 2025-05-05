# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_executable(timer_test timer_test.cc)
target_link_libraries(timer_test common)

install(TARGETS timer_test)

add_custom_command(
  TARGET timer_test POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:timer_test>" "$<TARGET_FILE:timer_test>.bin"
  COMMAND srec_cat "$<TARGET_FILE:timer_test>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:timer_test>.vmem" -vmem
  VERBATIM)
