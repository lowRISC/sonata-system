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

#include "../common/console.hh"
#include "../common/hyperram_perf_test.h"
#include "../common/timer-utils.hh"
#include "../common/uart-utils.hh"

#include "hyperram_memset.h"

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
int stripe_test(Capability<volatile uint32_t> hyperram_area, uint32_t initial_val, Log &log, bool report_time = false) {
  uint32_t failures      = 0;
  uint32_t cur_write_val = initial_val;
  uint32_t start_time    = get_mcycle();

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

  if (report_time) {
    uint32_t end_time = get_mcycle();
    log.print(" ({} cycles)...", end_time - start_time);
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

  // By writing the first word of the code again we can ensure that the code is
  // flushed out to the HyperRAM and will thus be coherent with instruction
  // fetching when the code is executed.
  hyperram_area[addr] = hyperram_area[addr];
}

// Writes a short function to a random area of hyperram and executes it checking
// for successful execution (see 'write_prog' for details on the function
// written).
int execute_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations, Log &log,
                 bool report_time = false) {
  uint32_t start_time = get_mcycle();
  int failures        = 0;

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

  if (report_time) {
    uint32_t end_time = get_mcycle();
    log.print(" ({} cycles)...", end_time - start_time);
  }

  return failures;
}

// Perform partial writes to addresses that may collide with the read buffer to check that
// read and write accesses are coherent.
int buffering_test(Capability<volatile uint32_t> hyperram_area, ds::xoroshiro::P64R32 &prng, int iterations) {
  const uint32_t burst_len = 32u;
  int failures             = 0;

  // Create an expectation buffer that we may update ourselves; this is a local buffer on the
  // stack and thus stored in the main system RAM.
  uint32_t exp_data[burst_len / 4];
  for (unsigned idx = 0u; idx < burst_len / 4; ++idx) {
    exp_data[idx] = prng();
  }

  for (int i = 0; i < iterations; ++i) {
    // Leave a small gap at the end of the test area, so that we may advance by half a burst length
    // and still perform a full length burst.
    uint32_t burst_addr = prng() % ((HyperramSize * 4) - 2 * burst_len);
    // Align to the start of a burst.
    burst_addr &= ~(burst_len - 1u);
    // With 50% probability, ensure that the buffer crosses a burst boundary, to make
    // the interaction of write and read less predictable.
    if (prng() & 1) {
      burst_addr += burst_len / 2;
    }

    // Decide upon a list of actions to be performed; the number of actions/iteration is pretty
    // arbitrary.
    const unsigned num_actions = 7u;
    uint32_t act_addr[num_actions];
    uint32_t act_data[num_actions];
    for (unsigned act = 0u; act < num_actions; ++act) {
      uint32_t wr_offset = prng() % burst_len;
      uint32_t wr_type   = prng() % 3;
      // Ensure that the write offset has natural alignment.
      wr_offset &= ~((1u << (wr_type)) - 1u);
      // Store the action type, offset and data compactly to keep the write operations
      // close together. We want to ensure that writes and reads occur simultaneously/close
      // together.
      act_addr[act] = wr_offset | (wr_type << 24);
      act_data[act] = prng();
    }

    // Randomise the word offset from which we read; read bursts are wrapping.
    uint32_t rd_off = prng() % (burst_len / 4);

    // ----- Avoid the use of randomisation after this point; timing would become predictable. -----

    // We need pointers to write to data of different sizes.
    volatile uint32_t *burst_wp = &hyperram_area[burst_addr / 4];
    volatile uint16_t *burst_hp = reinterpret_cast<volatile uint16_t *>(burst_wp);
    volatile uint8_t *burst_bp  = reinterpret_cast<volatile uint8_t *>(burst_wp);

    // Initialise the data at the target address.
    hyperram_copy_block(burst_wp, exp_data, burst_len);

    // Ensure that we have read data from an address, to provoke fetching of a burst of
    // read data from the HyperRAM. Check the first word is as expected.
    volatile uint32_t rd_data = burst_wp[rd_off];
    failures += (rd_data != exp_data[rd_off]);

    // Update the expectation according to the actions that we're about to perform.
    for (unsigned act = 0u; act < num_actions; ++act) {
      uint32_t wr_offset = act_addr[act] & ~0xff000000u;
      // Modify the affected word in each case. Note that `wr_offset` has natural alignment.
      unsigned sh = 8u * (wr_offset & 3u);
      uint32_t mask;
      switch (act_addr[act] >> 24) {
        case 0u:
          mask = 0xffu << sh;
          break;
        case 1u:
          mask = 0xffffu << sh;
          break;
        default:
          mask = ~0u;
          break;
      }
      exp_data[wr_offset >> 2] = (exp_data[wr_offset >> 2] & ~mask) | ((act_data[act] << sh) & mask);
    }

    // Perform the actions; keep this code short and fast to ensure that the write traffic
    // is still held within the controller interface. (Write traffic is flushed out to the
    // HyperRAM after a short while, rather than being held indefinitely.)
    for (unsigned act = 0u; act < num_actions; ++act) {
      uint32_t wr_offset = act_addr[act] & ~0xff000000u;
      uint32_t d         = act_data[act];
      switch (act_addr[act] >> 24) {
        case 0u:
          burst_bp[wr_offset] = (uint8_t)d;
          break;
        case 1u:
          burst_hp[wr_offset >> 1] = (uint16_t)d;
          break;
        default:
          burst_wp[wr_offset >> 2] = d;
          break;
      }
    }

    // ----- Avoid the use of randomisation before this point -----

    // Check the entire contents of the target buffer against our expectations.
    for (unsigned idx = 0u; idx < burst_len / 4; ++idx) {
      failures += (exp_data[idx] != burst_wp[idx]);
    }
  }

  return failures;
}

// Performance test to exercise burst transfers from/to the HyperRAM.
//
// - read buffers may be 'cleaned' before commencing the performance test, so that they have no
//   history of earlier read accesses.
// - source address and/or destination address will be randomised if not already set.
// - run time of each of the 'copy' and 'compare' operations in `mcycle` ticks may optionally be reported.
int perf_burst_test(Capability<volatile uint32_t> hyperram_area, Log &log, ds::xoroshiro::P64R32 &prng, size_t nbytes,
                    bool clean_first = true, bool report_times = true, uint32_t dst_addr = UINT32_MAX,
                    uint32_t src_addr = UINT32_MAX) {
  typedef volatile void *(*hr_copy_fn_t)(volatile uint32_t *, const volatile uint32_t *, size_t);
  int failures = 0;

  // Randomised word offsets.
  if (dst_addr == UINT32_MAX) {
    dst_addr = prng() & 0x3ffu;
  }
  if (src_addr == UINT32_MAX) {
    // Ensuring that the source and destination buffers cannot overlap.
    src_addr = 0x1000u - dst_addr;
  }

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
    // Do we need to clean all buffered read data out of the HyperRAM controller interface first?
    if (clean_first) {
      // Use 8KiB of data to ensure that 128 buffers of 64 bytes/burst can be cleaned out; this
      // should be more than enough for any current/future implementation.
      hyperram_cleaner(&hyperram_area[0x2000u], &hyperram_area[0x4000u]);
    }

    // Time the memory copy operation.
    uint32_t start_time = get_mcycle();
    if (code_in_hr) {
      hr_copy_ptr(d, s, nbytes);
    } else {
      hyperram_copy_block(d, s, nbytes);
    }

    // Read back and check the destination buffer; this becomes a significant part of the memory
    // traffic/execution time because presently there is no prefetching of read data, so we time
    // it separately.
    uint32_t cmp_start_time = get_mcycle();
    failures += (0 != hyperram_cmp_block(d, s, nbytes));

    if (report_times) {
      // Report the duration of the copy and compare operations separately.
      uint32_t end_time = get_mcycle();
      log.print("    copy:  {:#6d} - cmp: {:#6d} - total: {:#6d}...", cmp_start_time - start_time,
                end_time - cmp_start_time, end_time - start_time);
    }
  }

  return failures;
}

// Memory-writing tests to exercise the write coalescing behaviour.
//
// - tests all transfer sizes (byte, half-word, word and double-word)
// - exercises coalescing of partial word writes to form complete words
// - performs descending writes as well as ascending
int write_tests(Capability<volatile uint8_t> hyperram_b_area, Capability<volatile uint16_t> hyperram_h_area,
                Capability<volatile uint32_t> hyperram_w_area, Capability<volatile uint64_t> hyperram_d_area,
                ds::xoroshiro::P64R32 &prng, Log &log, WriteTestType test_type, int iterations = 1,
                uint32_t dst_addr = UINT32_MAX, uint32_t src_addr = UINT32_MAX) {
  int failures = 0;

  log.println("  Test type {}: {} iteration(s)", (int)test_type, iterations);

  for (int iter = 0; iter < iterations; ++iter) {
    // Choose a random start address and whether we are to intersperse reads.
    bool intersperse_reads = ((prng() & 1u) != 0u);
    uint32_t dst_off       = dst_addr;
    uint32_t src_off       = src_addr;
    // Choose source and destination addresses if not supplied.
    if (UINT32_MAX == dst_off) {
      dst_off = prng() & 0x3ffu;
    }
    if (UINT32_MAX == src_off) {
      src_off = prng() & 0x3ffu;
    }
    // Control area must not be overlapped by any write operation.
    const uint32_t ctrl_off = 0x800u;

    // Choose a random byte value to store, and a constrained length.
    uint32_t data = (uint8_t)prng();
    data |= data << 8;
    data |= data << 16;
    // Write at least 32 bytes and up to 95 into a buffer of 128.
    size_t len              = 0x20u + (prng() & 0x3fu);
    const uint32_t init_len = 0x80u;

    // Dword, word, hword and bytes aliases to the chosen target buffer.
    volatile uint64_t *dstd = &hyperram_d_area[(dst_off + 7u) >> 3];
    volatile uint32_t *dstw = &hyperram_w_area[(dst_off + 3u) >> 2];
    volatile uint16_t *dsth = &hyperram_h_area[(dst_off + 1u) >> 1];
    volatile uint8_t *dstb  = &hyperram_b_area[dst_off];
    // End of target buffer; these addresses are exclusive, i.e. the pointers reference
    // the first value _above_ the range to be modified.
    volatile uint64_t *edstd = &hyperram_d_area[(dst_off + len + 7u) >> 3];
    volatile uint32_t *edstw = &hyperram_w_area[(dst_off + len + 3u) >> 2];

    // The different tests have different alignment requirements so determine the offset
    // of the lowest address at which writing should be expected to occur.
    uint32_t align_mask;
    switch (test_type) {
      // Half word-aligned test cases.
      case WriteTestType_H:
      case WriteTestType_HB:
        align_mask = 1u;
        break;
      // Word-aligned test cases.
      case WriteTestType_WR:
      case WriteTestType_W:
      case WriteTestType_WD:
        align_mask = 3u;
        break;
      // Double word-aligned test cases.
      case WriteTestType_C:
      case WriteTestType_CD:
        align_mask = 7u;
        break;
      // Byte-aligned test cases.
      default:
        align_mask = 0u;
        break;
    }

    // Calculate the number of bytes that we have skipped to achieve natural alignment.
    // This works for most cases...
    uint32_t exp_start     = ((align_mask + 1u) - (dst_off & align_mask)) & align_mask;
    uint32_t written_start = exp_start;
    // ... but the descending transfers require special treatment.
    if (test_type == WriteTestType_WD) {
      exp_start     = ((dst_off + len + 3u) & ~3u) - (dst_off + len);
      written_start = exp_start + len;
    } else if (test_type == WriteTestType_CD) {
      exp_start     = ((dst_off + len + 7u) & ~7u) - (dst_off + len);
      written_start = exp_start + len;
    }

    // Initialise the control area.
    hyperram_memset_w(&hyperram_w_area[ctrl_off >> 2], 0u, init_len);

    // Initialise the target area, using data that we know should never be stored in the
    // ensuing memory writing code. Thus we maximise the chance of detecting any bytes
    // that are erroneously overwritten.
    hyperram_memset_b(dstb, ~data, init_len);

    uint32_t written_end = written_start;
    uint32_t bytes_left  = len;
    while (bytes_left > 0u) {
      // Decide upon a word-aligned offset from which to read; do this before the
      // memory writing because the random number generation is time-consuming and we want
      // to test the interaction of the read with the under-construction write burst.
      uint32_t rd_off  = prng() % init_len;
      size_t chunk_len = intersperse_reads ? (1u + (prng() % bytes_left)) : bytes_left;
      if (bytes_left <= align_mask) {  // Ensure the loop terminates.
        chunk_len = bytes_left;
      } else if (chunk_len < bytes_left) {  // Non-final chunk?
        // The next chunk must have natural alignment too; note that we could end up
        // with a chunk size of zero; memset routines accept that.
        chunk_len &= ~align_mask;
      }
      // Adjust our expectations of the area that should have been written after this chunk.
      if (test_type == WriteTestType_WD || test_type == WriteTestType_CD) {
        written_start -= chunk_len;
      } else {
        written_end += chunk_len;
      }

      // Perform the write operation.
      switch (test_type) {
        case WriteTestType_B:
          hyperram_memset_b(dstb, data, chunk_len);
          break;
        case WriteTestType_H:
          hyperram_memset_h(dsth, data, chunk_len);
          break;
        // HB test writes two bytes and a half-word, so it requires half-word alignment.
        case WriteTestType_HB:
          hyperram_memset_hb(dsth, data, chunk_len);
          break;
        case WriteTestType_W:
          hyperram_memset_w(dstw, data, chunk_len);
          break;
        case WriteTestType_WR:
          hyperram_memset_wr(dstw, data, chunk_len);
          break;
        case WriteTestType_WD:
          hyperram_memset_wd(edstw, data, chunk_len);
          break;
        case WriteTestType_C:
          hyperram_memset_c(dstd, data, chunk_len);
          break;
        default:
          assert(WriteTestType_CD == test_type);
          hyperram_memset_cd(edstd, data, chunk_len);
          break;
      }
      // Interject a read at this point, to test its interaction with the coalescing of
      // writes into bursts.
      if (intersperse_reads) {
        volatile uint8_t rd_data = hyperram_b_area[dst_off + rd_off];
        // This is the default value with which the target area was initialised.
        uint8_t exp_data = (uint8_t)~data;
        // Does the byte that we're checking lie within the range that should have been overwritten
        // at this point in the test?
        if (rd_off >= written_start && rd_off < written_end) exp_data = ~exp_data;
        failures += (rd_data != exp_data);
      }

      // Advance the destination pointers for the next chunk, maintaining natural alignment
      // in case this is not the final chunk of the transfer.
      dstb += chunk_len;
      dsth += chunk_len >> 1;
      dstw += chunk_len >> 2;
      dstd += chunk_len >> 3;
      edstw -= chunk_len >> 2;
      edstd -= chunk_len >> 3;
      bytes_left -= chunk_len;
    }

    // Read and check the control area; this primarily serves to ensure that we are checking
    // what was written to the HyperRAM itself, and not merely the contents of the internal
    // read buffer within the controller interface.
    for (uint32_t i = 0u; i < 0x80 / 4; i++) {
      failures += (hyperram_w_area[i + (ctrl_off >> 2)] != 0u);
    }

    // Read and check the entire target area.
    for (uint32_t i = 0u; i < len; i++) {
      // This is the default value with which the target area was initialised.
      uint8_t exp_data = (uint8_t)~data;
      // Does the byte that we're checking lie within the range that should have been overwritten?
      if (i >= exp_start && (i - exp_start) < len) exp_data = ~exp_data;
      failures += (hyperram_b_area[i + dst_off] != exp_data);
    }
  }

  return failures;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart0 = root.cast<volatile OpenTitanUart>();
  uart0.address()                          = UART_ADDRESS;
  uart0.bounds()                           = UART_BOUNDS;

  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);

  set_console_mode(log, CC_BOLD);
  log.print("\r\n\r\nGet hyped for hyperram!\r\n");
  set_console_mode(log, CC_RESET);

  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);

  // Default is word-based accesses, which is sufficient for most tests.
  Capability<volatile uint32_t> hyperram_area = root.cast<volatile uint32_t>();
  hyperram_area.address()                     = HYPERRAM_ADDRESS;
  hyperram_area.bounds()                      = HYPERRAM_BOUNDS;

  Capability<Capability<volatile uint32_t>> hyperram_cap_area = root.cast<Capability<volatile uint32_t>>();
  hyperram_cap_area.address()                                 = HYPERRAM_ADDRESS;
  hyperram_cap_area.bounds()                                  = HYPERRAM_BOUNDS;

  // We also want byte, hword and dword access for some tests.
  Capability<volatile uint8_t> hyperram_b_area  = root.cast<volatile uint8_t>();
  hyperram_b_area.address()                     = HYPERRAM_ADDRESS;
  hyperram_b_area.bounds()                      = HYPERRAM_BOUNDS;
  Capability<volatile uint16_t> hyperram_h_area = root.cast<volatile uint16_t>();
  hyperram_h_area.address()                     = HYPERRAM_ADDRESS;
  hyperram_h_area.bounds()                      = HYPERRAM_BOUNDS;
  Capability<volatile uint64_t> hyperram_d_area = root.cast<volatile uint64_t>();
  hyperram_d_area.address()                     = HYPERRAM_ADDRESS;
  hyperram_d_area.bounds()                      = HYPERRAM_BOUNDS;

  // Run indefinitely, soak testing until we observe one or more failures.
  int failures = 0;
  while (!failures) {
    const uint32_t burst_len = 32u;
    const bool report_times  = true;

    log.print("Running RND cap test...");
    failures += rand_cap_test(hyperram_area, hyperram_cap_area, prng, HyperramSize / 4);
    write_test_result(log, failures);

    log.print("Running RND data test...");
    failures += rand_data_test_full(hyperram_area, prng);
    write_test_result(log, failures);

    log.print("Running RND data & address test...");
    failures += rand_data_addr_test(hyperram_area, prng, HyperramSize / 4);
    write_test_result(log, failures);

    log.print("Running 0101 stripe test...");
    failures += stripe_test(hyperram_area, 0x55555555, log, report_times);
    write_test_result(log, failures);

    log.print("Running 1001 stripe test...");
    failures += stripe_test(hyperram_area, 0x99999999, log, report_times);
    write_test_result(log, failures);

    log.print("Running 0000_1111 stripe test...");
    failures += stripe_test(hyperram_area, 0x0F0F0F0F, log, report_times);
    write_test_result(log, failures);

    log.print("Running Execution test...");
    failures += execute_test(hyperram_area, prng, HyperramSize / 4, log, report_times);
    write_test_result(log, failures);

    // Performance test copies a significant chunk of data from one buffer to another.
    icache_invalidate();
    // Ensure that the icache is enabled.
    icache_enabled_set(true);
    log.println("Running performance test with icache enabled...");
    failures += perf_burst_test(hyperram_area, log, prng, 0x1000u);
    write_test_result(log, failures);

    // Executing with the icache disabled places more strain on the HyperRAM controller because
    // it will receive many more instruction fetches.
    icache_enabled_set(false);
    icache_invalidate();
    log.println("Running performance test with icache disabled...");
    failures += perf_burst_test(hyperram_area, log, prng, 0x1000u);
    write_test_result(log, failures);
    // Reinstate the normal icache operation.
    icache_enabled_set(true);

    // Run the same burst performance tests again, with all possible alignments, but also checking
    // the completed destination buffer against the source.
    log.print("Running alignment tests ");
    bool clean_first = false;
    do {
      clean_first = !clean_first;
      log.println(clean_first ? "with cleaning..." : "without cleaning...");
      for (uint32_t src_addr = 0x1000u; src_addr < 0x1000u + burst_len; src_addr += 4u) {
        for (uint32_t dst_addr = 0u; dst_addr < burst_len; dst_addr += 4u) {
          // We may want to investigate the impact of alignment upon performance; this is useful
          // in development/analysis...
          const bool report_times = false;  // ... but not required for regression testing.
          if (report_times) {
            log.print("  dst: {:#04x} src: {:#04x}...", dst_addr, src_addr & (burst_len - 1u));
          }
          failures += perf_burst_test(hyperram_area, log, prng, 0x1000u, clean_first, report_times, dst_addr, src_addr);
          if (report_times) {
            write_test_result(log, failures);
          }
        }
      }
    } while (clean_first);
    write_test_result(log, failures);

    // Buffering test checks the interaction of write traffic with buffered read data.
    log.print("Running buffering test...");
    failures += buffering_test(hyperram_area, prng, 0x1000u);
    write_test_result(log, failures);

    // Write tests exercise the write coalescing logic of the HyperRAM controller interface.
    log.println("Running write tests...");
    for (int test_type = WriteTestType_B; test_type <= WriteTestType_CD; ++test_type) {
      failures += write_tests(hyperram_b_area, hyperram_h_area, hyperram_area, hyperram_d_area, prng, log,
                              (WriteTestType)test_type, 0x400u);
    }
    log.print("  result...");
    write_test_result(log, failures);
  }

  // Report test failure.
  log.println("Test(s) failed: {}", failures);
  while (true) asm volatile("wfi");
}
