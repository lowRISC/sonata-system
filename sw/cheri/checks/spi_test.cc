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

#include "../common/sonata-devices.hh"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"

using namespace CHERI;

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  uint8_t write_data[256];
  uint8_t read_data[256];

  // Create a bounded capability to the UART
  UartPtr uart = uart_ptr(root, 0);
  SpiPtr spi   = spi_ptr(root, 2);

  SpiFlash spi_flash(spi);

  spi->init(false, false, true, 0);
  uart->init(BAUD_RATE);
  write_str(uart, "Hello World!\r\n");

  uint8_t jedec_id[3];
  spi_flash.read_jedec_id(jedec_id);
  write_str(uart, "JEDEC ID: ");
  for (auto id : jedec_id) {
    write_hex8b(uart, id);
    write_str(uart, " ");
  }
  write_str(uart, "\r\n");

  for (size_t i = 0; i < 256; ++i) {
    write_data[i] = i;
  }

  spi_flash.erase_sector(0);
  spi_flash.write_page(0, write_data);
  spi_flash.read(0, read_data, 256);

  write_str(uart, "Got first flash read:\r\n");
  for (auto data : read_data) {
    write_hex8b(uart, data);
    write_str(uart, " ");
  }
  write_str(uart, "\r\n");

  spi_flash.read(128, read_data, 256);

  write_str(uart, "Got second flash read:\r\n");
  for (auto data : read_data) {
    write_hex8b(uart, data);
    write_str(uart, " ");
  }
  write_str(uart, "\r\n");

  while (true) {
    asm("");
  }
}
