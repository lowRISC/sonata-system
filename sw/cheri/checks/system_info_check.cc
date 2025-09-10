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

// Read the 'most often unique'
static void dna_read(Capability<volatile uint32_t> sysinfo, uint64_t& dna) {
  // Initiate reading of the DNA value.
  sysinfo[8] = 1;
  // The DNA value is 57 bits in length and is read bit-serially.
  for (unsigned b = 0u; b < 57; ++b) {
    dna = (dna << 1) | (sysinfo[9] & 1u);
  }
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void* rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART.
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  // Create bounded capability to system info.
  Capability<volatile uint32_t> sysinfo = root.cast<volatile uint32_t>();
  sysinfo.address()                     = SYSTEM_INFO_ADDRESS;
  sysinfo.bounds()                      = SYSTEM_INFO_BOUNDS;

  uint32_t git_hash_0        = sysinfo[0];
  uint32_t git_hash_1        = sysinfo[1];
  uint32_t git_dirty         = sysinfo[2];
  uint32_t system_frequency  = sysinfo[3];
  uint32_t gpio_info         = sysinfo[4];
  uint32_t uart_info         = sysinfo[5];
  uint32_t i2c_info          = sysinfo[6];
  uint32_t spi_info          = sysinfo[7];
  uint32_t mem_size          = sysinfo[10];
  uint32_t hyperram_size     = sysinfo[11];
  uint32_t hyperram_tag_size = sysinfo[12];

  // Reading the 'DNA value' of the FPGA is more involved than a simple register read.
  uint64_t dna;
  dna_read(sysinfo, dna);

  uart->init(BAUD_RATE);
  write_str(uart, "Hash is equal to:\r\n");
  write_hex(uart, git_hash_0);
  write_hex(uart, git_hash_1);
  write_str(uart, " ");
  if (git_dirty) {
    write_str(uart, "DIRTY!");
  } else {
    write_str(uart, "clean");
  }
  write_str(uart, "\r\n");

  write_str(uart, "System clock frequency: 0x");
  write_hex(uart, system_frequency);
  write_str(uart, "\r\n");

  write_str(uart, "GPIO instances: 0x");
  write_hex(uart, gpio_info);
  write_str(uart, "\r\n");

  write_str(uart, "UART instances: 0x");
  write_hex(uart, uart_info);
  write_str(uart, "\r\n");

  write_str(uart, "I2C instances: 0x");
  write_hex(uart, i2c_info);
  write_str(uart, "\r\n");

  write_str(uart, "SPI instances: 0x");
  write_hex(uart, spi_info);
  write_str(uart, "\r\n");

  write_str(uart, "DNA: 0x");
  write_hex(uart, (uint32_t)(dna >> 32));
  write_hex(uart, (uint32_t)dna);
  write_str(uart, "\r\n");

  write_str(uart, "Mem size: 0x");
  write_hex(uart, mem_size);
  write_str(uart, "KiB\r\n");

  write_str(uart, "HyperRAM size: 0x");
  write_hex(uart, hyperram_size);
  write_str(uart, "KiB\r\n");

  write_str(uart, "Tagged HyperRAM size: 0x");
  write_hex(uart, hyperram_tag_size);
  write_str(uart, "KiB\r\n");

  char ch = '\n';
  while (true) {
    ch = uart->blocking_read();
    uart->blocking_write(ch);
  }
}
