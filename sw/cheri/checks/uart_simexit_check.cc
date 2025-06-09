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
extern "C" [[noreturn]] void entry_point(void* rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  uart->init(BAUD_RATE);
  write_str(uart, "Trying out simulation exit!\r\n");
  write_str(uart, "SSafe to exit simulator.\xd8\xaf\xfb\xa0\xc7\xe1\xa9\xd7");
  write_str(uart, "This should not be printed.\r\n");
  write_str(uart, "FAIL\r\n");
  while (true);
}
