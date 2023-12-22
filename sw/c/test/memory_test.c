// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>

#include "dev_access.h"
#include "sonata_system.h"
#include "timer.h"

#define TEST_DATA (0xDEADFEEF)
#define TEST_SIZE (5)

int test_array[TEST_SIZE];

int pass() {
  while(true);
  return 0;
}

int fail() {
  while(true);
  return -1;
}

int main(void) {
  // RAM test
  for (int i = 0; i < TEST_SIZE; i++) {
    // Write data
    test_array[i] = TEST_DATA + i;
  }
  for (int i = 0; i < TEST_SIZE; i++) {
    // Read back data and check it is correct
    if (test_array[i] != TEST_DATA + i) {
      return fail();
    }
  }

  // Peripheral test
  DEV_WRITE(TIMER_BASE + TIMER_MTIMECMP_REG, TEST_DATA);
  if (DEV_READ(TIMER_BASE + TIMER_MTIMECMP_REG) != TEST_DATA) {
    return fail();
  }

  return pass();
}
