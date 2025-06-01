// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "fmt.hh"

#include "../hal/uart.hh"
#include "platform.hh"
#include "sonata_system.h"

struct WriteUart {
  Uart serial{platform::Uart::Uart0, SYSCLK_FREQ};
  void write(const char* buf, size_t n) {
    while (n--) {
      serial.write_byte(*buf++);
    }
  }
};

using Log = reisfmt::Fmt<WriteUart>;

WriteUart uart;
Log log(uart);
