# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set(SRC_BOOT boot_loader.cc)

get_filename_component(NAME ${SRC_BOOT} NAME_WE)

add_executable(${NAME} ${SRC_BOOT} boot.S)
target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})

add_custom_command(
  TARGET ${NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
  COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
  VERBATIM
)

install(TARGETS ${NAME})
install(FILES "$<TARGET_FILE:${NAME}>.vmem" DESTINATION share)
