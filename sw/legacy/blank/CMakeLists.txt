# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
add_executable(blank blank.S)

add_custom_command(
  TARGET blank POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:blank>" "$<TARGET_FILE:blank>.bin"
  COMMAND srec_cat "$<TARGET_FILE:blank>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:blank>.vmem" -vmem
  VERBATIM)
