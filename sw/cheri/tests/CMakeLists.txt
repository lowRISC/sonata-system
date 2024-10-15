# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
set(TESTS
  test_runner.cc
)

foreach(TEST ${TESTS})
  get_filename_component(NAME ${TEST} NAME_WE)

  add_executable(${NAME} ${TEST})
  target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES} "${reisfmt_SOURCE_DIR}/include")
  target_link_libraries(${NAME} common)

  install(TARGETS ${NAME})
endforeach()
