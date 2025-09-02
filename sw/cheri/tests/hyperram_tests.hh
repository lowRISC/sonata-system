// Copyright lowRISC Contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
// clang-format off
#include "../../common/defs.h"
#include <ds/xoroshiro.h>
#include <cheri.hh>
// clang-format on
#include <platform-uart.hh>

#include "../common/console.hh"
#include "../common/hyperram_perf_test.h"
#include "../common/sonata-devices.hh"
#include "../common/uart-utils.hh"
#include "test_runner.hh"
#include "../common/console.hh"

using namespace CHERI;

/*
 * Configures the number of iteratios.
 * It can be overwride with a compilation flag
 */
#ifndef HYPERRAM_TEST_ITERATIONS
#define HYPERRAM_TEST_ITERATIONS 1
#endif

/*
 * Configures the percentage of the flash should be tested.
 * It can be overwride with a compilation flag
 */
#ifndef TEST_COVERAGE_AREA
// Test only n/1024 of the total memory to be fast enough for Verilator.
//
// Note: we are deliberately using power-of-two quantities here because of the addressing
//       limitations of CHERIoT capabilities. There is 8MiB available but we must keep the
//       testing short, so we choose to use just ca. 0.2%
#define TEST_COVERAGE_AREA 2
#endif
static_assert(TEST_COVERAGE_AREA <= 1024, "TEST_COVERAGE_AREA Should be less than 1024");

#define TEST_BLOCK_SIZE 256
// Size of mapped, tag-capable portion of HyperRAM, in 32-bit words.
#define HYPERRAM_TAG_SIZE (HYPERRAM_TAG_BOUNDS / 4)

/*
 * Compute the number of addresses that will be tested.
 * We mask the LSB 8bits to makes sure it is aligned.
 */
#define HYPERRAM_TEST_SIZE (uint32_t)((HYPERRAM_TAG_SIZE * TEST_COVERAGE_AREA / 0x400u) & ~0xFF)

/*
 * Write random values to a block of memory (size given by 'TEST_BLOCK_SIZE'
 * global constant). Reads them all back and checks read values matched written
 * values.
 */
static int rand_data_test_block(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng,
                                uint32_t start_hr_addr) {
  std::array<uint32_t, TEST_BLOCK_SIZE> write_values;

  uint32_t index = 0;
  for (auto &write_value : write_values) {
    write_value                          = prng();
    hyperram_area[index + start_hr_addr] = write_value;
    index++;
  }

  int failures = 0;
  for (int i = 0; i < write_values.size(); ++i) {
    if (hyperram_area[i + start_hr_addr] != write_values[i]) {
      ++failures;
    }
  }

  return failures;
}

/*
 * Writes random values to all of hyperram and reads back to check read values
 * matched written values. It does this one 'TEST_BLOCK_SIZE' sized block at a
 * time.
 */
int rand_data_test_full(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng) {
  int failures = 0;
  for (uint32_t addr = 0; addr < HYPERRAM_TEST_SIZE; addr += TEST_BLOCK_SIZE) {
    failures += rand_data_test_block(hyperram_area, prng, addr);
  }

  return failures;
}

/*
 * Writes a random value to a random address then reads it back to check the
 * written value matches the read value.
 */
int rand_data_addr_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations) {
  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t rand_addr;
    uint32_t rand_val;
    uint32_t read_val;

    rand_addr = prng() % HYPERRAM_TEST_SIZE;
    rand_val  = prng();

    hyperram_area[rand_addr] = rand_val;
    read_val                 = hyperram_area[rand_addr];

    if (read_val != rand_val) {
      failures += 1;
    }
  }

  return failures;
}

/*
 * Writes a random value to a random address and then writes a capability for
 * that random address to another random location. Reads back the capability and
 * then reads back the value via the capability to check it matches what was
 * originally written.
 */
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
      rand_index = prng() % HYPERRAM_TEST_SIZE;

      // Capability is double word in size.
      rand_cap_index = prng() % (HYPERRAM_TEST_SIZE / 2);
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

/*
 * Writes a 32-bit value in every location in hyperram and then reads back to
 * check read values match written values. The values written alternate between
 * 'initial_val' and the inversion of 'initial_val'.
 */
int stripe_test(Capability<volatile uint32_t> hyperram_area, uint32_t initial_val) {
  uint32_t failures      = 0;
  uint32_t cur_write_val = initial_val;

  for (uint32_t addr = 0; addr < HYPERRAM_TEST_SIZE; addr++) {
    hyperram_area[addr] = cur_write_val;
    cur_write_val       = ~cur_write_val;
  }

  uint32_t cur_expected_val = initial_val;

  for (uint32_t addr = 0; addr < HYPERRAM_TEST_SIZE; addr++) {
    uint32_t read_value = hyperram_area[addr];
    if (read_value != cur_expected_val) {
      failures++;
    }

    cur_expected_val = ~cur_expected_val;
  }

  return failures;
}

typedef void *(*test_fn_t)(uint32_t *);
/*
 * Gets a function pointer to an address in hyperram, expects to be called with
 * a PC capability that provides execution in hyperram. 'addr' is relative to
 * the base of hyperram.
 */
extern "C" test_fn_t get_hyperram_fn_ptr(uint32_t addr);

void write_prog(Capability<volatile uint32_t> hyperram_area, uint32_t addr) {
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

  // By writing the first word of the code again we can ensure that the code is
  // flushed out to the HyperRAM and will thus be coherent with instruction
  // fetching when the code is executed.
  hyperram_area[addr] = hyperram_area[addr];
}

/*
 * Writes a short function to a random area of hyperram and executes it checking
 * for successful execution (see 'write_prog' for details on the function
 * written).
 */
int execute_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations) {
  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t prog_addr = prng() % (HYPERRAM_TEST_SIZE - 5);

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

// Performance test to exercise burst transfers from/to the HyperRAM.
int perf_burst_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, size_t nbytes) {
  typedef volatile void *(*hr_copy_fn_t)(volatile uint32_t *, const volatile uint32_t *, size_t);
  int failures = 0;

  // Randomised word offsets.
  uint32_t dst_addr = prng() & 0x3ffu;
  uint32_t src_addr = 0x1000u - dst_addr;

  volatile uint32_t *d = &hyperram_area[dst_addr];
  volatile uint32_t *s = &hyperram_area[src_addr];

  // Complete the source buffer with complete words; it doesn't matter that we may write a few extra bytes.
  const uint32_t whole_words = nbytes / 4u;
  for (unsigned idx = 0u; idx <= whole_words; idx++) {
    hyperram_area[src_addr + idx] = prng();
  }

  // Copy the code into the HyperRAM, using itself.
  const uint32_t prog_addr = 0x903u;
  hyperram_copy_block(&hyperram_area[prog_addr], (volatile uint32_t *)hyperram_copy_block, hyperram_copy_size);
  hr_copy_fn_t hr_copy_ptr = (hr_copy_fn_t)get_hyperram_fn_ptr(HYPERRAM_ADDRESS + (prog_addr * 4));

  for (unsigned code_in_hr = 0; code_in_hr < 2; ++code_in_hr) {
    if (code_in_hr) {
      hr_copy_ptr(d, s, nbytes);
    } else {
      hyperram_copy_block(d, s, nbytes);
    }

    // Read back and check the destination buffer.
    failures += (0 != hyperram_cmp_block(d, s, nbytes));
  }

  return failures;
}

void hyperram_tests(CapRoot root, Log &log) {
  // Unfortunately it is not possible to construct a capability that covers exactly the 8MiB range
  // of the HyperRAM because of the encoding limitations of the CHERIoT capabilities, but here we
  // are only concerned with testing a much smaller portion of the address range anyway.
  const uint32_t hr_bounds = HYPERRAM_TEST_SIZE * 4u;
  auto hyperram_area       = hyperram_ptr(root);

  Capability<Capability<volatile uint32_t>> hyperram_cap_area = root.cast<Capability<volatile uint32_t>>();
  hyperram_cap_area.address()                                 = HYPERRAM_ADDRESS;
  hyperram_cap_area.bounds()                                  = hr_bounds;

  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);

  for (size_t i = 0; i < HYPERRAM_TEST_ITERATIONS; i++) {
    log.println("\nrunning hyperram_test: {} \\ {}", i, HYPERRAM_TEST_ITERATIONS - 1);
    set_console_mode(log, CC_PURPLE);
    log.println("(HYPERRAM_TEST_SIZE: {:#08x})", HYPERRAM_TEST_SIZE);
    set_console_mode(log, CC_RESET);
    bool test_failed = false;
    int failures     = 0;

    log.print("  Running RND cap test...");
    failures = rand_cap_test(hyperram_area, hyperram_cap_area, prng, HYPERRAM_TEST_SIZE);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running RND data test...");
    failures = rand_data_test_full(hyperram_area, prng);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running RND data & address test...");
    failures = rand_data_addr_test(hyperram_area, prng, HYPERRAM_TEST_SIZE);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running 0101 stripe test...");
    failures = stripe_test(hyperram_area, 0x55555555);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running 1001 stripe test...");
    failures = stripe_test(hyperram_area, 0x99999999);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running 0000_1111 stripe test...");
    failures = stripe_test(hyperram_area, 0x0F0F0F0F);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running Execution test...");
    failures = execute_test(hyperram_area, prng, HYPERRAM_TEST_SIZE);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running Performance test...");
    failures = perf_burst_test(hyperram_area, prng, 0x1000u);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    check_result(log, !test_failed);
  }
}
