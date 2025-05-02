// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>
#include <stdlib.h>

#include "../common/log.hh"
extern "C" {
#include "dev_access.h"
#include "sonata_system.h"
#include "timer.h"
}

extern char _heap_size[];
#define TEST_DATA (0xDEADBEEF)
// Total ram size minus space for .text and .rodata
#define TEST_SIZE_BYTES ((uint32_t)_heap_size / 4)
#define TEST_SIZE_WORDS TEST_SIZE_BYTES / 4

int pass() {
  log.println("Test Passed");

  while (true);
  return 0;
}

int fail() {
  log.println("Test Failed");

  while (true);
  return -1;
}

int memory_test(uint32_t *pointer, size_t size) {
  log.print("Memory_test testing {:#x} bytes  at addr: {:#x} ...", size * sizeof(uint32_t), uintptr_t(pointer));
  // Simple word write/read test of most of the RAM; some RAM is used by the
  // code itself, global static and stack...
  for (size_t i = 0; i < size; i++) {
    pointer[i] = TEST_DATA + i;
  }
  // Read back data and check it is correct
  for (size_t i = 0; i < size; i++) {
    if (pointer[i] != TEST_DATA + i) {
      log.println("Failed");
      return -1;
    }
  }

  // Byte read/write test
  uint8_t *btest = (uint8_t *)pointer;
  for (size_t i = 0; i < size; i++) {
    btest[i * 4 + (i & 3)] ^= i;
  }
  // Read back data and check it is correct
  for (size_t i = 0; i < size; i++) {
    uint32_t orig = TEST_DATA + i;
    uint32_t mod  = orig ^ (uint8_t)i << (8 * (i & 3));
    if (pointer[i] != mod) {
      log.println("Failed");
      return -1;
    }
  }
  log.println("Passed");
  return 0;
}

int main(void) {
  log.println("\n>>> Starting {} ", __FILE__);

  // Allocate a chunk of memory, run the test and free the pointer.
  uint32_t *test_array1 = (uint32_t *)malloc(TEST_SIZE_BYTES);
  if (test_array1 == NULL) {
    log.println("Failed to allocate memory");
    return fail();
  }
  int ret = memory_test(test_array1, TEST_SIZE_WORDS);
  free(test_array1);
  if (ret == -1) {
    return fail();
  }

  // Allocate another chunk of memory, Check that the pointers point to the same address, run the test, and let the
  // pointer allocated.
  uint32_t *test_array2 = (uint32_t *)malloc(TEST_SIZE_BYTES);
  if (test_array2 == NULL) {
    log.println("Failed to allocate memory");
    return fail();
  }
  if (test_array1 != test_array2) {
    log.println("Failed: Second malloc should reuse freed memory.");
    return fail();
  }
  ret = memory_test(test_array2, TEST_SIZE_WORDS);
  if (ret == -1) {
    return fail();
  }

  // Allocate another chunk of memory, Check that the pointers point to different addresses, run the test, and free the
  // pointer.
  uint32_t *test_array3 = (uint32_t *)malloc(TEST_SIZE_BYTES);
  if (test_array3 == NULL) {
    log.println("Failed to allocate memory");
    return fail();
  }
  if (test_array3 == test_array2) {
    log.println("Failed: Allocated unfree memory.");
    return fail();
  }
  ret = memory_test(test_array3, TEST_SIZE_WORDS);
  free(test_array2);
  free(test_array3);

  return pass();
}
