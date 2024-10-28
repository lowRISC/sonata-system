# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13)
include(FetchContent)

FetchContent_Declare(CHERIOT_RTOS
  GIT_REPOSITORY    https://github.com/lowRISC/CHERIoT-RTOS
  GIT_TAG           8fbbda7b0ee528764892e1add9b539f9ec9064b0
)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cheriot_toolchain.cmake")
endif()

if(NOT DEFINED CHERIOT_RTOS_SDK)
  FetchContent_Populate(CHERIOT_RTOS)
  set(CHERIOT_RTOS_SDK "${cheriot_rtos_SOURCE_DIR}/sdk")
endif()

FetchContent_Declare(REISFMT
  GIT_REPOSITORY    https://github.com/engdoreis/reisfmt
  GIT_TAG           4ce04e1bc88d37ad359da051b91b4071f740c3d8
)
FetchContent_Populate(REISFMT)

project(sonata_system_cheriot_sw LANGUAGES C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CHERIOT_SDK_INCLUDES
  "${CHERIOT_RTOS_SDK}/include/c++-config"
  "${CHERIOT_RTOS_SDK}/include/libc++"
  "${CHERIOT_RTOS_SDK}/include"
  "${CHERIOT_RTOS_SDK}/include/platform/sunburst"
  "${CHERIOT_RTOS_SDK}/include/platform/generic-riscv"
)

add_subdirectory(boot)
add_subdirectory(common)
add_subdirectory(checks)
add_subdirectory(tests)
add_subdirectory(error_leds)
