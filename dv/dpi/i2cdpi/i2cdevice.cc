// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdarg.h>
#include <stdio.h>

#include "i2cdevice.hh"

// Logging utility function.
void i2cdevice::logText(const char *fmt, ...) {
  if (logging) {
    va_list va;
    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);
  }
}
