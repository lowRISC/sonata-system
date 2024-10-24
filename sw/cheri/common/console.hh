/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "fmt.hh"
#include "sonata-devices.hh"
#include "../../common/defs.h"
#include <platform-uart.hh>

#include "console.hh"

#define CC_BOLD "1"
#define CC_RED "31"
#define CC_GREEN "32"
#define CC_RESET "0"

struct WriteUart {
  UartPtr uart;
  void write(const char* buf, size_t n) {
    while (n--) {
      uart->blocking_write(*buf++);
    }
  }
};

using Log = reisfmt::Fmt<WriteUart>;

[[maybe_unused]] static void set_console_mode(Log& log, const char* cc) { log.print("\x1b[{}m", cc); }

[[maybe_unused]] static void write_test_result(Log& log, int failures) {
  if (failures == 0) {
    set_console_mode(log, CC_GREEN);
    log.println("PASS!");
  } else {
    set_console_mode(log, CC_RED);
    log.println("FAIL!");
  }
  set_console_mode(log, CC_RESET);
}
