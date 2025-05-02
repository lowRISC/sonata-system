# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_library(common OBJECT sonata_system.c usbdev.c uart.c timer.c rv_plic.c gpio.c i2c.c pwm.c spi.c crt0.S
  ../../../vendor/cheriot_debug_module/tb/prog/syscalls.c
  cpp_runtime.cc
)
target_include_directories(common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

