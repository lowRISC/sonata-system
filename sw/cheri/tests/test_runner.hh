/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "../common/sonata-devices.hh"
#include "../common/console.hh"

[[noreturn]] static void finish_running(Log& log, const char* message) {
  log.println(message);

  while (true) asm volatile("wfi");
}

[[maybe_unused]] static void check_result(Log& log, bool result) {
  if (result) {
    return;
  }
  finish_running(log, "Test(s) Failed");
}
