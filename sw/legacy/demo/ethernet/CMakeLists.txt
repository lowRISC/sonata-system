# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set(CMAKE_C_FLAGS "-O2")

include(FetchContent)
FetchContent_Declare(
  lwip
  GIT_REPOSITORY https://github.com/lwip-tcpip/lwip.git
  GIT_TAG STABLE-2_2_0_RELEASE
)
FetchContent_Populate(lwip)

set(LWIP_DIR ${lwip_SOURCE_DIR})

set (LWIP_INCLUDE_DIRS
    ${LWIP_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lwip
)

include(${LWIP_DIR}/src/Filelists.cmake)
target_link_libraries(lwipcore common)

add_executable(ethernet main.c lwip/sys.c ksz8851.c)

target_link_libraries(ethernet lwipcore common)

target_include_directories(ethernet PRIVATE ${LWIP_INCLUDE_DIRS})

install(TARGETS ethernet)

add_custom_command(
  TARGET ethernet POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:ethernet>" "$<TARGET_FILE:ethernet>.bin"
  COMMAND srec_cat "$<TARGET_FILE:ethernet>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:ethernet>.vmem" -vmem
  VERBATIM)
