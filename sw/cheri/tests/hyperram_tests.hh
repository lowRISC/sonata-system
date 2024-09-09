
// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <ds/xoroshiro.h>
#include <cheri.hh>
#include <functional>
#include <platform-uart.hh>
#include "../../common/defs.h"
#include "../common/console-utils.hh"
#include "../common/sonata-peripherals.hh"
#include "test_runner.hh"
#include "../common/ostream.hh"

using namespace CHERI;

/*
 * Configures the number of iteratios.
 * It can be overwride with a compilation flag
 */
#ifndef TEST_ITERATIONS
#define TEST_ITERATIONS 1
#endif

/*
 * Configures the percentage of the flash should be tested.
 * It can be overwride with a compilation flag
 */
#ifndef TEST_COVERAGE_AREA
// Test only 1% of the total memory to be fast enough for varilator.
#define TEST_COVERAGE_AREA 1
#endif
 _Static_assert(TEST_COVERAGE_AREA <= 100, "TEST_COVERAGE_AREA Should be less than 100");

#define  TEST_BLOCK_SIZE  256
#define  HYPERRAM_SIZE  (1024 * 1024) / 4

/*
 * Compute the number of addresses that will be tested.
 * We mask the LSB 8bits to makes sure it is aligned.
 */
#define HYPERRAN_TEST_SIZE \
  (uint32_t)((HYPERRAM_SIZE * TEST_COVERAGE_AREA / 100) & ~0xFF)

/* 
 * Write random values to a block of memory (size given by 'TEST_BLOCK_SIZE'
 * global constant). Reads them all back and checks read values matched written
 * values.
 */
static int rand_data_test_block(Capability<volatile uint32_t> hyperram_area,
    ds::xoroshiro::P64R32 &prng, uint32_t start_hr_addr) {
  std::array<uint32_t,TEST_BLOCK_SIZE> write_values;

  uint32_t index = 0;
  for (auto& write_value : write_values) {
    write_value = prng();
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
int rand_data_test_full(Capability<volatile uint32_t> hyperram_area,
    ds::xoroshiro::P64R32 &prng) {

  int failures = 0;
  for (uint32_t addr = 0; addr < HYPERRAN_TEST_SIZE; addr += TEST_BLOCK_SIZE) {
    failures += rand_data_test_block(hyperram_area, prng, addr);
  }

  return failures;
}

/*
 * Writes a random value to a random address then reads it back to check the
 * written value matches the read value.
 */
int rand_data_addr_test(Capability<volatile uint32_t> hyperram_area,
    ds::xoroshiro::P64R32 &prng, int iterations) {

  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t rand_addr;
    uint32_t rand_val;
    uint32_t read_val;

    rand_addr = prng() % HYPERRAN_TEST_SIZE;
    rand_val = prng();

    hyperram_area[rand_addr] = rand_val;
    read_val = hyperram_area[rand_addr];

    if (read_val != rand_val) {
      failures += 1;
    }
  }

  return failures;
}

/* 
 * Writes a random value to a random address and then writes a capability for
 * that random address to another random location. Reads back the capability and
 * then reads back the value via the capability to check it matches what we was
 * originally written.
 */
int rand_cap_test(Capability<volatile uint32_t> hyperram_area,
    Capability<Capability<volatile uint32_t>> hyperram_cap_area,
    ds::xoroshiro::P64R32 &prng, int iterations) {

  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t rand_index;
    uint32_t rand_cap_index;
    uint32_t rand_val;
    uint32_t read_val;

    Capability<volatile uint32_t> write_cap;
    Capability<volatile uint32_t> read_cap;

    do {
      rand_index = prng() % HYPERRAN_TEST_SIZE;

      // Capability is double word in size.
      rand_cap_index = prng() % (HYPERRAN_TEST_SIZE / 2);
    } while (rand_index / 2 == rand_cap_index);

    rand_val = prng();

    write_cap = hyperram_area;
    write_cap.address() += (rand_index * 4);
    write_cap.bounds() = 4;

    hyperram_area[rand_index] = rand_val;
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
int stripe_test(Capability<volatile uint32_t> hyperram_area,
    uint32_t initial_val) {
  uint32_t failures = 0;
  uint32_t cur_write_val = initial_val;

  for (uint32_t addr = 0; addr < HYPERRAN_TEST_SIZE; addr++) {
    hyperram_area[addr] = cur_write_val;
    cur_write_val = ~cur_write_val;
  }

  uint32_t cur_expected_val = initial_val;

  for (uint32_t addr = 0; addr < HYPERRAN_TEST_SIZE; addr++) {
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
 * Gets a function pointer to an address in hyperram, expectes to be called with
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

  hyperram_area[addr] = 0xdeadc2b7;
  hyperram_area[addr + 1] = 0xeef28293;
  hyperram_area[addr + 2] = 0x00552023;
  hyperram_area[addr + 3] = 0x00000517;
  hyperram_area[addr + 4] = 0x8082;

  asm volatile ("fence.i" : : : "memory");
}

/*
 * Writes a short function to a random area of hyperram and executes it checking
 * for successful execution (see 'write_prog' for details on the function
 * written).
 */
int execute_test(Capability<volatile uint32_t> hyperram_area,
    ds::xoroshiro::P64R32 &prng, int iterations) {

  int failures = 0;

  for (int i = 0; i < iterations; ++i) {
    uint32_t prog_addr = prng() % (HYPERRAN_TEST_SIZE - 5);

    write_prog(hyperram_area, prog_addr);

    uint32_t test_int = 0x0;
    void *test_ptr;

    test_fn_t test_fn = get_hyperram_fn_ptr(HYPERRAM_ADDRESS + (prog_addr * 4));
    test_ptr = test_fn(&test_int);

    if (test_int != 0xdeadbeef) {
      failures++;
    }

    if (!__builtin_cheri_tag_get(test_ptr)) {
      failures++;
    }

    uint32_t expected_ptr_addr = HYPERRAM_ADDRESS + 0xC + (prog_addr * 4);
    uint32_t test_ptr_addr = __builtin_cheri_address_get(test_ptr);

    if (test_ptr_addr != expected_ptr_addr) {
      failures++;
    }
  }

  return failures;
}

void hyperram_tests(CapRoot root, LOG::OStream& console)
{
  auto hyperram_area = hyperram_ptr(root);

  Capability<Capability<volatile uint32_t>> hyperram_cap_area =
    root.cast<Capability<volatile uint32_t>>();
	hyperram_cap_area.address() = HYPERRAM_ADDRESS;
	hyperram_cap_area.bounds()  = HYPERRAM_BOUNDS;

  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);

  for (size_t i = 0; i < TEST_ITERATIONS; i++) {
    console << LOG::endl << "running hyperram_test: " << i + 1 << "\\" << TEST_ITERATIONS << LOG::endl
            << "Hyperram size: " << HYPERRAN_TEST_SIZE << LOG::endl;

    int failures = 0;
    console << "Running RND cap test...";
    failures +=
      rand_cap_test(hyperram_area, hyperram_cap_area, prng, HYPERRAN_TEST_SIZE);
    write_test_result(console, failures);

    console << "Running RND data test...";
    failures = rand_data_test_full(hyperram_area, prng);
    write_test_result(console, failures);

    console << "Running RND data & address test...";
    failures = rand_data_addr_test(hyperram_area, prng, HYPERRAN_TEST_SIZE);
    write_test_result(console, failures);

    console << "Running 0101 stripe test...";
    failures = stripe_test(hyperram_area, 0x55555555);
    write_test_result(console, failures);

    console << "Running 1001 stripe test...";
    failures = stripe_test(hyperram_area, 0x99999999);
    write_test_result(console, failures);

    console << "Running 0000_1111 stripe test...";
    failures = stripe_test(hyperram_area, 0x0F0F0F0F);
    write_test_result(console, failures);

    console << "Running Execution test...";
    failures = execute_test(hyperram_area, prng, HYPERRAN_TEST_SIZE);
    write_test_result(console, failures);
  }
}
