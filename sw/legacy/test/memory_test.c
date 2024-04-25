// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This test is useful when making changes to the memory sub-system.
// It first performs a basic word write/read test of the RAM.
// It then proceeds to test byte read/writes.
// Then it writes to a peripheral register and checks the value that is read back.
// At the moment it passes and fails using an infinite while loop so that you can
// check the result using a wave output from simulation.

#include <stdbool.h>

#include "dev_access.h"
#include "sonata_system.h"
#include "timer.h"

#define TEST_DATA (0xDEADBEEF)
#define TEST_SIZE ((224*1024)/4)

static uint32_t test_array[TEST_SIZE];

int pass() {
  puts("Passed");

  while(true);
  return 0;
}

int fail() {
  puts("Failed");

  while(true);
  return -1;
}

int main(void) {
  uart_init(DEFAULT_UART);

  putstr("memory_test running...");
  puthex(TEST_SIZE * 4);
  puts(" bytes");

  // Simple word write/read test of most of the RAM; some RAM is used by the
  // code itself, global static and stack...
  for (int i = 0; i < TEST_SIZE; i++) {
    test_array[i] = TEST_DATA + i;
  }
  // Read back data and check it is correct
  for (int i = 0; i < TEST_SIZE; i++) {
    if (test_array[i] != TEST_DATA + i) {
      return fail();
    }
  }

  // Byte read/write test
  uint8_t *btest = (uint8_t*)test_array;
  for (int i = 0; i < TEST_SIZE; i++) {
    btest[i * 4 + (i & 3)] ^= i;
  }
  // Read back data and check it is correct
  for (int i = 0; i < TEST_SIZE; i++) {
    uint32_t orig = TEST_DATA + i;
    uint32_t mod  = orig ^ (uint8_t)i << (8 * (i & 3));
    if (test_array[i] != mod) {
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
