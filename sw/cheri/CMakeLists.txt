cmake_minimum_required(VERSION 3.13)
include(FetchContent)

FetchContent_Declare(CHERIOT_RTOS
  GIT_REPOSITORY    https://github.com/microsoft/cheriot-rtos
  GIT_TAG           c3c169ae729d7293bb8d36b0a901869b3e4b36f6
)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cheriot_toolchain.cmake")
endif()

if(NOT DEFINED CHERIOT_RTOS_SDK)
  FetchContent_Populate(CHERIOT_RTOS)
  set(CHERIOT_RTOS_SDK "${cheriot_rtos_SOURCE_DIR}/sdk")
endif()

project(sonata_system_cheriot_sw LANGUAGES C CXX ASM)

set(CHERIOT_SDK_INCLUDES
  "${CHERIOT_RTOS_SDK}/include/c++-config"
  "${CHERIOT_RTOS_SDK}/include/libc++"
  "${CHERIOT_RTOS_SDK}/include"
  "${CHERIOT_RTOS_SDK}/include/platform/generic-riscv"
)

add_subdirectory(common)
add_subdirectory(tests)