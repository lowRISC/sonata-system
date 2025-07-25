/**
  * Copyright lowRISC contributors.
  * Licensed under the Apache License, Version 2.0, see LICENSE for details.
  * SPDX-License-Identifier: Apache-2.0
  */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>

#include <stdbool.h>
#include <array>

#include <platform-timer.hh>
#include <platform-uart.hh>

#include "../common/sonata-devices.hh"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"
#include "../common/console.hh"

using namespace CHERI;

int print_capability(void* ptr)
{
    int *t = static_cast<int*>(cheri_tag_get(ptr));
    return 0;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};
  int32_t value_one = 80;
  int32_t value_two = 27;
  int32_t result;
  asm("mv a0, %0" : : "r"(value_one) : "a0");
  asm("mv a1, %0" : : "r"(value_two) : "a1");
  // Add the values in a0 and a1, result is in a0
  asm("add a0, a0, a1");

  // Extract the result from a0 back into the C variable 'result'
  asm("mv %0, a0" : "=r"(result) : : "a0");
  uint8_t write_data[256];
  uint8_t read_data[256];
  read_data[0] = value_one;
  read_data[1] = value_two;
  read_data[2] = result;

  // Create a bounded capability to the UART
  UartPtr uart = uart_ptr(root, 0);
  uart->init(BAUD_RATE);
  int i = 0;
  for (auto data : read_data) {
    write_hex8b(uart, data);
    write_str(uart, " ");
    if (i > 1){
        break;
    }
    else{
        i++;
    }
  }

  write_str(uart, "\n");
  Capability<volatile uint32_t> result_cap;
  Capability<volatile uint32_t> test_cap = root.cast<volatile uint32_t>();
  test_cap.address() = 0x12;
  test_cap.bounds() = 100;
  asm("mv a0, %0" : : "r"(test_cap) : "a0");
  asm("mv %0, a0" : "=r"(result_cap) : : "a0");

  int tag = print_capability(static_cast<void*>(&test_cap));
  write_hex8b(uart, tag);
  write_str(uart, "\n");
  if (tag){
    write_str(uart, "Tag is set\n");
  }
  else{
    write_str(uart, "Tag is not set\n");
  }
}