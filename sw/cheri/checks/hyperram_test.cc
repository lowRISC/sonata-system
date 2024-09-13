/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
#include <ds/xoroshiro.h>
#include <cheri.hh>
// clang-format on
#include <platform-uart.hh>

#include "../common/console-utils.hh"
#include "../common/uart-utils.hh"

using namespace CHERI;

const int RandTestBlockSize = 256;
const int HyperramSize      = (1024 * 1024) / 4;

// Write random values to a block of memory (size given by 'RandTestBlockSize'
// global constant). Reads them all back and checks read values matched written
// values.
int rand_data_test_block(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng,
                         uint32_t start_hr_addr) {
  uint32_t write_values[RandTestBlockSize];
  uint32_t read_values[RandTestBlockSize];

  for (int i = 0; i < RandTestBlockSize; ++i) {
    write_values[i] = prng();
  }

  for (int i = 0; i < RandTestBlockSize; ++i) {
    hyperram_area[i + start_hr_addr] = write_values[i];
  }

  for (int i = 0; i < RandTestBlockSize; ++i) {
    read_values[i] = hyperram_area[i + start_hr_addr];
  }

  int failures = 0;

  for (int i = 0; i < RandTestBlockSize; ++i) {
    if (read_values[i] != write_values[i]) {
      ++failures;
    }
  }

  return failures;
}

// Writes random values to all of hyperram and reads back to check read values
// matched written values. It does this one 'RandTestBlockSize' sized block at a
// time.
int rand_data_test_full(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng) {
  int failures = 0;
  for (uint32_t addr = 0; addr < HyperramSize; addr += RandTestBlockSize) {
    failures += rand_data_test_block(hyperram_area, prng, addr);
  }

  return failures;
}

// Writes a random value to a random address then reads it back to check the
// written value matches the read value.
int rand_data_addr_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations) {
  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t rand_addr;
    uint32_t rand_val;
    uint32_t read_val;

    rand_addr = prng() % HyperramSize;
    rand_val  = prng();

    hyperram_area[rand_addr] = rand_val;
    read_val                 = hyperram_area[rand_addr];

    if (read_val != rand_val) {
      failures += 1;
    }
  }

  return failures;
}

// Writes a random value to a random address and then writes a capability for
// that random address to another random location. Reads back the capability and
// then reads back the value via the capability to check it matches what we was
// originally written.
int rand_cap_test(Capability<volatile uint32_t> hyperram_area,
                  Capability<Capability<volatile uint32_t>> hyperram_cap_area, ds::xoroshiro::P64R32 &prng,
                  int iterations) {
  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t rand_index;
    uint32_t rand_cap_index;
    uint32_t rand_val;
    uint32_t read_val;

    Capability<volatile uint32_t> write_cap;
    Capability<volatile uint32_t> read_cap;

    do {
      rand_index = prng() % HyperramSize;

      // Capability is double word in size.
      rand_cap_index = prng() % (HyperramSize / 2);
    } while (rand_index / 2 == rand_cap_index);

    rand_val = prng();

    write_cap = hyperram_area;
    write_cap.address() += (rand_index * 4);
    write_cap.bounds() = 4;

    hyperram_area[rand_index]         = rand_val;
    hyperram_cap_area[rand_cap_index] = write_cap;

    asm volatile("" : : : "memory");

    read_cap = hyperram_cap_area[rand_cap_index];
    read_val = *read_cap;

    if (read_val != rand_val) {
      failures++;
    }
  }

  return failures;
}

// Writes a 32-bit value in every location in hyperram and then reads back to
// check read values match written values. The values written alternate between
// 'initial_val' and the inversion of 'initial_val'.
int stripe_test(Capability<volatile uint32_t> hyperram_area, uint32_t initial_val) {
  uint32_t failures      = 0;
  uint32_t cur_write_val = initial_val;

  for (uint32_t addr = 0; addr < HyperramSize; addr++) {
    hyperram_area[addr] = cur_write_val;
    cur_write_val       = ~cur_write_val;
  }

  uint32_t cur_expected_val = initial_val;

  for (uint32_t addr = 0; addr < HyperramSize; addr++) {
    uint32_t read_value = hyperram_area[addr];
    if (read_value != cur_expected_val) {
      failures++;
    }

    cur_expected_val = ~cur_expected_val;
  }

  return failures;
}

typedef void *(*test_fn_t)(uint32_t *);
// Gets a function pointer to an address in hyperram, expectes to be called with
// a PC capability that provides execution in hyperram. 'addr' is relative to
// the base of hyperram.
extern "C" test_fn_t get_hyperram_fn_ptr(uint32_t addr);

void write_prog(Capability<volatile uint32_t> &hyperram_area, uint32_t addr) {
  // Avoid use of global data (as it's currently broken in the test environment)
  // by writing program data directly here.

  // Test program, writes 0xdeadbeef to capability provided in first argument
  // (ca0) and returns a capability pointing to the middle of the function
  // (offset + 0xC from function start).
  //
  // li t0, 0xdeadbeef # Expands to two 32-bit instructions
  // csw t0, 0(ca0)
  // auipcc ca0, 0
  // cret

  hyperram_area[addr]     = 0xdeadc2b7;
  hyperram_area[addr + 1] = 0xeef28293;
  hyperram_area[addr + 2] = 0x00552023;
  hyperram_area[addr + 3] = 0x00000517;
  hyperram_area[addr + 4] = 0x8082;

  asm volatile("fence.i" : : : "memory");
}

// Writes a short function to a random area of hyperram and executes it checking
// for successful execution (see 'write_prog' for details on the function
// written).
int execute_test(Capability<volatile uint32_t> &hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations) {
  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t prog_addr = prng() % (HyperramSize - 5);

    write_prog(hyperram_area, prog_addr);

    uint32_t test_int = 0x0;
    void *test_ptr;

    test_fn_t test_fn = get_hyperram_fn_ptr(HYPERRAM_ADDRESS + (prog_addr * 4));
    test_ptr          = test_fn(&test_int);

    if (test_int != 0xdeadbeef) {
      failures++;
    }

    if (!__builtin_cheri_tag_get(test_ptr)) {
      failures++;
    }

    uint32_t expected_ptr_addr = HYPERRAM_ADDRESS + 0xC + (prog_addr * 4);
    uint32_t test_ptr_addr     = __builtin_cheri_address_get(test_ptr);

    if (test_ptr_addr != expected_ptr_addr) {
      failures++;
    }
  }

  return failures;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  uart->init(BAUD_RATE);
  set_console_mode(uart, CC_BOLD);
  write_str(uart, "\r\n\r\nGet hyped for hyperram!\r\n");
  set_console_mode(uart, CC_RESET);

  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);

  Capability<volatile uint32_t> hyperram_area = root.cast<volatile uint32_t>();
  hyperram_area.address()                     = HYPERRAM_ADDRESS;
  hyperram_area.bounds()                      = HYPERRAM_BOUNDS;

  Capability<Capability<volatile uint32_t>> hyperram_cap_area = root.cast<Capability<volatile uint32_t>>();
  hyperram_cap_area.address()                                 = HYPERRAM_ADDRESS;
  hyperram_cap_area.bounds()                                  = HYPERRAM_BOUNDS;

  while (true) {
    int failures = 0;
    write_str(uart, "Running RND cap test...");
    failures += rand_cap_test(hyperram_area, hyperram_cap_area, prng, HyperramSize / 4);
    write_test_result(uart, failures);

    write_str(uart, "Running RND data test...");
    failures = rand_data_test_full(hyperram_area, prng);
    write_test_result(uart, failures);

    write_str(uart, "Running RND data & address test...");
    failures = rand_data_addr_test(hyperram_area, prng, HyperramSize / 4);
    write_test_result(uart, failures);

    write_str(uart, "Running 0101 stripe test...");
    failures = stripe_test(hyperram_area, 0x55555555);
    write_test_result(uart, failures);

    write_str(uart, "Running 1001 stripe test...");
    failures = stripe_test(hyperram_area, 0x99999999);
    write_test_result(uart, failures);

    write_str(uart, "Running 0000_1111 stripe test...");
    failures = stripe_test(hyperram_area, 0x0F0F0F0F);
    write_test_result(uart, failures);

    write_str(uart, "Running Execution test...");
    failures = execute_test(hyperram_area, prng, HyperramSize / 4);
    write_test_result(uart, failures);
  }
}
