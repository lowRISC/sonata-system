# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_executable(boot boot.c)
target_link_libraries(boot common)

add_custom_command(
  TARGET boot POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:boot>" "$<TARGET_FILE:boot>.bin"
  COMMAND srec_cat "$<TARGET_FILE:boot>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:boot>.vmem" -vmem
  VERBATIM)
