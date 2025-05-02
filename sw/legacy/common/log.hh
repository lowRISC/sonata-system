// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "fmt.hh"

extern "C" {
#include "sonata_system.h"
}

struct WriteUart {
  uart_t uart;
  WriteUart(uart_t uart) : uart(uart) { uart_init(DEFAULT_UART); }

  void write(const char* buf, size_t n) {
    while (n--) {
      uart_out(uart, *buf++);
    }
  }
};

using Log = reisfmt::Fmt<WriteUart>;

WriteUart uart{DEFAULT_UART};
Log log(uart);
