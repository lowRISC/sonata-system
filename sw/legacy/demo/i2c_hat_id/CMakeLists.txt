# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_executable(i2c_hat_id i2c_hat_id.c)

# pull in core dependencies and additional i2c hardware support
target_link_libraries(i2c_hat_id common)

install(TARGETS i2c_hat_id)

add_custom_command(
  TARGET i2c_hat_id POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:i2c_hat_id>" "$<TARGET_FILE:i2c_hat_id>.bin"
  COMMAND srec_cat "$<TARGET_FILE:i2c_hat_id>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:i2c_hat_id>.vmem" -vmem
  VERBATIM)
