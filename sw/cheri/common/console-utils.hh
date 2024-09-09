/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "../../common/defs.h"
#include "uart-utils.hh"
#include <platform-uart.hh>
#include "../common/ostream.hh"

#define CC_BOLD "1"
#define CC_RED "31"
#define CC_GREEN "32"
#define CC_RESET "0"


[[maybe_unused]] static void set_console_mode(volatile OpenTitanUart *uart,
                      const char *cc) {
  write_str(uart, "\x1b[");
  write_str(uart, cc);
  write_str(uart, "m");
}

[[maybe_unused]] static void write_test_result(volatile OpenTitanUart *uart,
                       int failures) {
  if (failures == 0) {
    set_console_mode(uart, CC_GREEN);
    write_str(uart, "PASS!\n");
  } else {
    set_console_mode(uart, CC_RED);
    write_str(uart, "FAIL!\n");
  }
  set_console_mode(uart, CC_RESET);
}

    [[maybe_unused]] static void write_test_result(LOG::OStream& console, int failures) {
      if (failures == 0) {
        LOG::set_console_mode(console, LOG::consoleGreen);
        console << "PASS!" << LOG::endl;
      } else {
        LOG::set_console_mode(console, LOG::consoleRed);
        console << "FAIL!" << LOG::endl;
      }
      LOG::set_console_mode(console, LOG::consoleReset);
    }
