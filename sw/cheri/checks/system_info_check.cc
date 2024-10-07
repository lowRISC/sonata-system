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
#include <cheri.hh>
// clang-format on
#include <platform-uart.hh>
#include "../common/uart-utils.hh"

using namespace CHERI;

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void* rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART.
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  // Create bounded capability to system info.
  Capability<uint32_t> sysinfo = root.cast<uint32_t>();
  sysinfo.address()            = SYSTEM_INFO_ADDRESS;
  sysinfo.bounds()             = SYSTEM_INFO_BOUNDS;

  uint32_t git_hash  = sysinfo[0];
  uint32_t git_dirty = sysinfo[1];

  uart->init(BAUD_RATE);
  write_str(uart, "Hash is equal to:\r\n");
  write_hex(uart, git_hash);
  write_str(uart, " ");
  if (git_dirty) {
    write_str(uart, "DIRTY!");
  } else {
    write_str(uart, "clean");
  }
  write_str(uart, "\r\n");

  char ch = '\n';
  while (true) {
    ch = uart->blocking_read();
    uart->blocking_write(ch);
  }
}
