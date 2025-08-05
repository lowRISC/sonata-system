# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
set(INSTR_TESTS
    capability_modification_test_harness.cc
)

foreach(INSTR_TESTS ${INSTR_TESTS})
  get_filename_component(NAME ${INSTR_TESTS} NAME_WE)

  add_executable(${NAME} ${INSTR_TESTS})
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES} "${reisfmt_SOURCE_DIR}/include")
  target_link_libraries(${NAME} common)

  add_custom_command(
    TARGET ${NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
    COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
    VERBATIM
  )

  install(TARGETS ${NAME})
endforeach()

