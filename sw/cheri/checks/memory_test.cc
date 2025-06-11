// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This test is useful when making changes to the memory sub-system.
// It first performs a basic word write/read test of the RAM.
// It then proceeds to test byte read/writes.
// Then it writes to a peripheral register and checks the value that is read back.
// At the moment it passes and fails using an infinite while loop so that you can
// check the result using a wave output from simulation.

// clang-format off
#include "../../common/defs.h"
#include <cheri.hh>
// clang-format on

#include <stdbool.h>
#include <array>

#include <platform-timer.hh>
#include <platform-uart.hh>

#include "../common/console.hh"
#include "../common/uart-utils.hh"

using namespace CHERI;

#define TEST_DATA (0xDEADBEEF)
// Total ram size minus space for .text and .rodata
#define TEST_SIZE_BYTES (0x1F000 - 0x8000)
#define TEST_SIZE_WORDS TEST_SIZE_BYTES / 4

static std::array<uint32_t, TEST_SIZE_WORDS> test_array;

// Test has passed; does not return.
static void pass(Log &log) {
  log.println("Passed");
  log.println("All tests finished");

  while (true) {
    asm volatile("wfi");
  };
}

// Test has failed; does not return.
static void fail(Log &log) {
  log.println("Tests(s) Failed");

  while (true) {
    asm volatile("wfi");
  };
}

// Return the current value of the 64-bit CPU cycle counter.
inline uint64_t ibex_mcycle_read(void) {
  uint32_t cycle_low    = 0;
  uint32_t cycle_high   = 0;
  uint32_t cycle_high_2 = 0;
  asm volatile(
      "1:"
      "  csrr %0, mcycleh;"  // Read `mcycleh`.
      "  csrr %1, mcycle;"   // Read `mcycle`.
      "  csrr %2, mcycleh;"  // Read `mcycleh` again.
      "  bne  %0, %2, 1b;"   // Try again if `mcycle` overflowed before
                             // reading `mcycleh`.
      : "=r"(cycle_high), "=r"(cycle_low), "=r"(cycle_high_2)
      :);
  return (uint64_t)cycle_high << 32 | cycle_low;
}

extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};
  const bool logging = false;

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart0 = root.cast<volatile OpenTitanUart>();
  uart0.address()                          = UART_ADDRESS;
  uart0.bounds()                           = UART_BOUNDS;

  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);
  log.println("memory_test running...{:x} bytes", TEST_SIZE_BYTES);

  // Having now completed UART logging, record the value of the CPU cycle counter
  // at the start of the test.
  uint64_t start_time = ibex_mcycle_read();

  // Simple word write/read test of most of the RAM; some RAM is used by the
  // code itself, global static and stack...
  for (int i = 0; i < TEST_SIZE_WORDS; i++) {
    test_array[i] = TEST_DATA + i;
  }
  // Read back data and check it is correct
  for (int i = 0; i < TEST_SIZE_WORDS; i++) {
    if (test_array[i] != TEST_DATA + i) {
      fail(log);
    }
  }

  // Byte read/write test
  uint8_t *btest = (uint8_t *)test_array.data();
  for (int i = 0; i < TEST_SIZE_WORDS; i++) {
    btest[i * 4 + (i & 3)] ^= i;
  }
  // Read back data and check it is correct
  for (int i = 0; i < TEST_SIZE_WORDS; i++) {
    uint32_t orig = TEST_DATA + i;
    uint32_t mod  = orig ^ (uint8_t)i << (8 * (i & 3));
    if (test_array[i] != mod) {
      fail(log);
    }
  }

  // Peripheral test; just use the 64-bit `mtimecmp` as per the legacy
  // `memory_test` from which this derives.
  static constexpr size_t ClintMtimecmp = 0x4000U;
  TimerPtr timer                        = timer_ptr(root);
  volatile uint32_t *mtimecmp           = &timer[ClintMtimecmp >> 2];
  mtimecmp[0]                           = TEST_DATA;
  mtimecmp[1]                           = ~TEST_DATA;
  if (TEST_DATA != mtimecmp[0] || ~TEST_DATA != mtimecmp[1]) {
    fail(log);
  }

  // Report the run time since this is useful for investigating the performance
  // impact of any changes to the memory system.
  uint64_t end_time     = ibex_mcycle_read();
  uint64_t elapsed_time = end_time - start_time;
  if (logging) {
    log.println("Start time: {:08x}{:08x}", (uint32_t)start_time >> 32, (uint32_t)start_time);
    log.println("End   time: {:08x}{:08x}", (uint32_t)end_time >> 32, (uint32_t)end_time);
  }
  // We expect to be able to report a cycle count far below 2^32.
  if (elapsed_time >> 32) {
    fail(log);
  }
  log.println("Elapsed time: {:u} cycles", (uint32_t)(end_time - start_time));

  pass(log);
}
