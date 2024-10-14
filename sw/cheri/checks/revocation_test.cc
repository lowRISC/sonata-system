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

#define CPU_FREQ_HZ (40'000'000)
#include "../../common/defs.h"

/**
 * OpenTitan UART
 */
class OpenTitanUart {
  typedef uint32_t RegisterType;

  [[maybe_unused]] RegisterType intrState;
  [[maybe_unused]] RegisterType intrEnable;
  [[maybe_unused]] RegisterType intrTest;
  [[maybe_unused]] RegisterType alertTest;
  [[maybe_unused]] RegisterType ctrl;
  [[maybe_unused]] RegisterType status;
  [[maybe_unused]] RegisterType rData;
  RegisterType wData;
  [[maybe_unused]] RegisterType fifoCtrl;
  RegisterType fifoStatus;
  [[maybe_unused]] RegisterType ovrd;
  [[maybe_unused]] RegisterType val;
  [[maybe_unused]] RegisterType timeoutCtrl;

 public:
  void init() volatile {
    // NCO = 2^20 * baud rate / cpu frequency
    const uint32_t nco = (((uint64_t)BAUD_RATE << 20) / CPU_FREQ_HZ);
    // Set the baud rate and enable transmit & receive
    ctrl = (nco << 16) | 0b11;
  };

  bool can_write() volatile { return (fifoStatus & 0xff) < 32; };

  /**
   * Write one byte, blocking until the byte is written.
   */
  void blocking_write(uint8_t byte) volatile {
    while (!can_write()) {
    }
    wData = byte;
  }

  void write_str(const char *str) volatile {
    while (*str) {
      blocking_write(*str);
      ++str;
    }
  }

  void write_hex(uint32_t n) volatile {
    char str_buf[9];

    for (int i = 0; i < 8; ++i) {
      if ((n & 0xf) < 10) {
        str_buf[7 - i] = (n & 0xf) + '0';
      } else {
        str_buf[7 - i] = (n & 0xf) + 'a' - 10;
      }

      n >>= 4;
    }
    str_buf[8] = 0;

    write_str(str_buf);
  }

  void write_hex8b(uint8_t n) volatile {
    char str_buf[3];

    for (int i = 0; i < 2; ++i) {
      if ((n & 0xf) < 10) {
        str_buf[1 - i] = (n & 0xf) + '0';
      } else {
        str_buf[1 - i] = (n & 0xf) + 'a' - 10;
      }

      n >>= 4;
    }
    str_buf[2] = 0;

    write_str(str_buf);
  }
};

#define READ_DELAY (16)

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = 0x80100000;
  uart.bounds()                           = 0x1000;

  unsigned int size_of_revocation_tags = 0x4000;  // 16 KiB
  unsigned int number_of_words         = size_of_revocation_tags / 4;

  Capability<volatile uint32_t> revocation_tags = root.cast<volatile uint32_t>();
  revocation_tags.address()                     = 0x30000000;
  revocation_tags.bounds()                      = size_of_revocation_tags;

  uart->init();
  uart->write_str("Hello Revocation!\r\n");

  uint32_t tmp;

  for (uint32_t i = 0; i < READ_DELAY; ++i) {
    ((volatile uint32_t *)revocation_tags)[i] = i;
    uart->write_str(".");
  }
  uart->write_str("\r\n");

  for (uint32_t i = 0; i < number_of_words - READ_DELAY; ++i) {
    ((volatile uint32_t *)revocation_tags)[i + READ_DELAY] = i + READ_DELAY;
    tmp                                                    = ((volatile uint32_t *)revocation_tags)[i];
    uart->write_str("Read: ");
    uart->write_hex(tmp);
    if (i == tmp) {
      uart->write_str(" success\r\n");
    } else {
      uart->write_str(" FAIL\r\n");
    }
  }

  for (uint32_t i = 0; i < READ_DELAY; ++i) {
    uint32_t index = number_of_words - READ_DELAY + i;
    tmp            = ((volatile uint32_t *)revocation_tags)[index];
    uart->write_str("Read: ");
    uart->write_hex(tmp);
    if (index == tmp) {
      uart->write_str(" success\r\n");
    } else {
      uart->write_str(" FAIL\r\n");
    }
  }

  while (true) {
    asm("");
  }
}
