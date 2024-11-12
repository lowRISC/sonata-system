/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

#include <cheri.hh>

using namespace CHERI;

#include "../common/uart-utils.hh"

#include <platform-uart.hh>

#define READ_DELAY (16)

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  unsigned int size_of_revocation_tags = 0x0800;  // 2 KiB
  unsigned int number_of_words         = size_of_revocation_tags / 4;

  Capability<volatile uint32_t> revocation_tags = root.cast<volatile uint32_t>();
  revocation_tags.address()                     = 0x30000000;
  revocation_tags.bounds()                      = size_of_revocation_tags;

  uart->init(BAUD_RATE);
  write_str(uart, "Hello Revocation!\r\n");

  uint32_t tmp;

  for (uint32_t i = 0; i < READ_DELAY; ++i) {
    ((volatile uint32_t *)revocation_tags)[i] = i;
    write_str(uart, ".");
  }
  write_str(uart, "\r\n");

  for (uint32_t i = 0; i < number_of_words - READ_DELAY; ++i) {
    ((volatile uint32_t *)revocation_tags)[i + READ_DELAY] = i + READ_DELAY;
    tmp                                                    = ((volatile uint32_t *)revocation_tags)[i];
    write_str(uart, "Read: ");
    write_hex(uart, tmp);
    if (i == tmp) {
      write_str(uart, " success\r\n");
    } else {
      write_str(uart, " FAIL\r\n");
    }
  }

  for (uint32_t i = 0; i < READ_DELAY; ++i) {
    uint32_t index = number_of_words - READ_DELAY + i;
    tmp            = ((volatile uint32_t *)revocation_tags)[index];
    write_str(uart, "Read: ");
    write_hex(uart, tmp);
    if (index == tmp) {
      write_str(uart, " success\r\n");
    } else {
      write_str(uart, " FAIL\r\n");
    }
  }

  while (true) {
    asm("");
  }
}
