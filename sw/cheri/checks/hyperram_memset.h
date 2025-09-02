/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

enum WriteTestType {
  WriteTestType_B = 0,
  WriteTestType_H,
  WriteTestType_HB,
  WriteTestType_W,
  WriteTestType_WR,
  WriteTestType_WD,
  WriteTestType_C,
  WriteTestType_CD
};

// Memory-writing routines; these all mimic the ISO C function `memset` except that they have
// a constraint of 'natural alignment' upon the buffer address and have very specific
// implementations to ensure defined traffic for testing the write coalescing/buffering logic
// of the HyperRAM controller interface.
extern "C" void hyperram_memset_b(volatile uint8_t *dst, int c, size_t n);
extern "C" void hyperram_memset_h(volatile uint16_t *dst, int c, size_t n);
extern "C" void hyperram_memset_hb(volatile uint16_t *dst, int c, size_t n);
extern "C" void hyperram_memset_w(volatile uint32_t *dst, int c, size_t n);
extern "C" void hyperram_memset_wr(volatile uint32_t *dst, int c, size_t n);
extern "C" void hyperram_memset_wd(volatile uint32_t *dst, int c, size_t n);
extern "C" void hyperram_memset_c(volatile uint64_t *dst, int c, size_t n);
extern "C" void hyperram_memset_cd(volatile uint64_t *dst, int c, size_t n);

// Utility function to return the address at which an exception occurred.
extern "C" void *get_mepcc(void);
