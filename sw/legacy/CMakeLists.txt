# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.12)
include(FetchContent)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/gcc_toolchain.cmake")
endif()


FetchContent_Declare(REISFMT
  GIT_REPOSITORY    https://github.com/engdoreis/reisfmt
  GIT_TAG           4ce04e1bc88d37ad359da051b91b4071f740c3d8
)
FetchContent_Populate(REISFMT)
add_library(reisfmt INTERFACE)
target_include_directories(reisfmt INTERFACE "${reisfmt_SOURCE_DIR}/include")
target_compile_options(reisfmt INTERFACE "-O3")

# For lsp
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(sonata_system_sw LANGUAGES CXX C ASM)

if(CMAKE_BUILD_TYPE STREQUAL "")
  get_property(helpstring CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "${helpstring}" FORCE)
endif()

option(SIM_CTRL_OUTPUT
       "Send string output to simulator control rather than UART")

if(SIM_CTRL_OUTPUT)
  add_compile_definitions(SIM_CTRL_OUTPUT)
endif()

add_subdirectory(common)
add_subdirectory(demo)
add_subdirectory(blank)
add_subdirectory(test)
add_subdirectory(timer_test)
