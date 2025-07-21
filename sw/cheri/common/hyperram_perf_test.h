/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>

// Helper code for manipulation of the instruction cache.
extern "C" void icache_enabled_set(bool enabled);
// Helper code to ensure that there is no stale code in the instruction cache after writing
// new code into memory.
//
// void icache_invalidate(void);
#define icache_invalidate() asm volatile("fence.i");

// Size in bytes of the `hyperram_copy_block` code. This function is used to copy itself into
// the HyperRAM memory before execution from there, so the size of the code is required.
extern uint32_t hyperram_copy_size;

// Memory-copying routine in the manner of `memcpy` but with a word alignment constraint on the
// base addresses of the source and destination buffers, and handwritten in manner that exercises
// the burst read/write behaviour of the HyperRAM controller interface and permits the code to be
// copied into the HyperRAM itself for execution.
extern "C" void hyperram_copy_block(volatile uint32_t *d, const volatile uint32_t *s, size_t nbytes);

// Utility function to clean any buffered read data out of the HyperRAM controller interface.
// This may be important for performance measurements or for ensuring that read requests actually/
// access the HyperRAM device rather than returning buffered data.
//
// For the current implementation it is sufficient to perform just 4 reads that are separated by
// 32 bytes, but for future-proofing this routine should be called with at least
// 128 x 64 bytes = 8192 bytes of data.
extern "C" void hyperram_cleaner(const volatile void *start, const volatile void *end);

// Memory comparison function in the manner of `memcmp` but with the same specific requirements
// and constraints as `hyperram_copy_block.`
extern "C" int hyperram_cmp_block(const volatile uint32_t *d, const volatile uint32_t *s, size_t nbytes);
