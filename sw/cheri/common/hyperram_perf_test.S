# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

// This is a memory copying function intended to exercise both the instruction memory system
// and the load/store data performance by performing a number of back-to-back transactions.
//
// Any permutation of the following may be in the HyperRAM:
// (i)   source data buffer
// (ii)  destination data buffer
// (iii) instruction sequence
//
// To ensure that data loads and stores are back-to-back, and to facilitate copying of this
// code into HyperRAM for execution, we employ handwritten assembler.
//
// Functions are aligned to 32-byte boundaries on the expectation that this is a multiple of
// the burst length and thus the code starts at the beginning of a read buffer. This just makes
// the performance measurements more robust against changes elsewhere in the code. All
// alignments of source and destination buffer are checked in other tests.

  .section .text, "ax", @progbits

  .option norvc
  .globl hyperram_copy_block
  .p2align 5
  .type hyperram_copy_block,@function

// Controlled, performant copying routine that is position-independent and may be copied into
// the HyperRAM for in-place execution.
//
// ca0 -> destination (word-aligned)
// ca1 -> source (word-aligned)
// a2  =  number of bytes to copy
hyperram_copy_block:
  srl   a3, a2, 5
  andi  a2, a2, 31  // 0-31 bytes remaining after 32-byte loop.
  sll   a3, a3, 5
  add   a3, a1, a3
  bgeu  a1, a3, copy32fix

  // 32 bytes/iteration.
copy32:
  clw   a5,   (ca1)
  clw   t0,  4(ca1)
  clw   t1,  8(ca1)
  clw   t2, 12(ca1)
  csw   a5,   (ca0)
  csw   t0,  4(ca0)
  csw   t1,  8(ca0)
  csw   t2, 12(ca0)
  clw   a5, 16(ca1)
  clw   t0, 20(ca1)
  clw   t1, 24(ca1)
  clw   t2, 28(ca1)
  cincoffset ca1, ca1, 32
  csw   a5, 16(ca0)
  csw   t0, 20(ca0)
  csw   t1, 24(ca0)
  csw   t2, 28(ca0)
  cincoffset ca0, ca0, 32
  bltu  a1, a3, copy32
copy32fix:
  srl   a3, a2, 2
  andi  a2, a2, 3  // 0-3 bytes remaining after 4-byte loop.
  sll   a3, a3, 2
  add   a3, a1, a3
  bgeu  a1, a3, copy4fix

  // 4 bytes/iteration.
copy4:
  clw   a5, (ca1)
  cincoffset ca1, ca1, 4
  csw   a5, (ca0)
  cincoffset ca0, ca0, 4
  bltu  a1, a3, copy4
copy4fix:
  beqz  a2, copyret
  li    a3, 2  // This constant allows conditional branching on 1, 2 or > 2

  // 1 byte/iteration; 0-3 bytes left, so fully unrolled.
copy1:
  clbu  a5,  (ca1)
  csb   a5,  (ca0)
  bltu  a2, a3, copyret
  clbu  a5, 1(ca1)
  csb   a5, 1(ca0)
  beq   a2, a3, copyret
  clbu  a5, 2(ca1)
  csb   a5, 2(ca0)
copyret:
  cret
hyperram_copy_end:

// Size of the copying code, in bytes; the copying routine is itself copied into HyperRAM in
// order to exercise instruction and load/store traffic simultaneously.
  .globl hyperram_copy_size
hyperram_copy_size:
  .long hyperram_copy_end-hyperram_copy_block


  .globl hyperram_cmp_block
  .p2align 5
  .type hyperram_cmp_block,@function

// Compare the contents of two word-aligned buffers.
//
// ca0 -> first buffer (word-aligned)
// ca1 -> second buffer (word-aligned)
// a2  =  number of bytes to compare
hyperram_cmp_block:
  cincoffset csp, csp, -16
  csc   cs0,   (csp)
  csc   cs1,  8(csp)
  andi  a3, a2, 15
  xor   a2, a2, a3
  add   a2, a1, a2
cmp16:
  bgeu a1, a2, cmp16done
  // This implementation is rather specific to the current buffering scheme
  // of the HyperRAM controller and is designed to minimise thrashing on the
  // LSU read buffer; read 16 bytes of data from one buffer into the register
  // file and only then start reading from the other and performing comparisons.
  //
  // Note: We could replace this with a more conventional implementation now
  // and compare a larger number of bytes per iteration.
  clw   a4,   (ca1)
  clw   a5,  4(ca1)
  clw   t0,  8(ca1)
  clw   t1, 12(ca1)  // End of 16 bytes from second buffer.
  clw   s0,   (ca0)
  clw   s1,  4(ca0)
  cincoffset ca1, ca1, 16
  bne   a4, s0, cmp_mismatch
  clw   a4,  8(ca0)
  bne   a5, s1, cmp_mismatch
  clw   a5, 12(ca0)
  cincoffset ca0, ca0, 16
  bne   t0, a4, cmp_mismatch
  beq   t1, a5, cmp16
cmp_mismatch:
  li    a0, 1
  j     cmp_leave
cmp16done:
  // We're not too concerned with performance here, so just do the remainder
  // as individual bytes, but unroll and order instructions to minimise the
  // loop overhead and limit load-use stalls.
  //
  // a3 = number of bytes remaining.
  beqz  a3, cmp_match
  li    t0, 2 // This constant allows conditional branch on 1, 2 or > 2
cmp1:
  clbu  a4, (ca1)
  clbu  s0, (ca0)
  bltu  a3, t0, cmp_last0
  clbu  a5, 1(ca1)  // Safe to read the next byte pair.
  clbu  s1, 1(ca0)
  bne   a4, s0, cmp_mismatch
  beq   a3, t0, cmp_last1
  bne   a5, s1, cmp_mismatch
  clbu  a4, 2(ca1)
  clbu  s0, 2(ca0)
  cincoffset ca1, ca1, 3
  cincoffset ca0, ca0, 3
  bne   a4, s0, cmp_mismatch
  addi  a3, a3, -3
  bnez  a3, cmp1
  j     cmp_match
cmp_last0:
  bne   a5, s1, cmp_mismatch
  j     cmp_match
cmp_last1:
  bne   a4, s0, cmp_mismatch
cmp_match:
  li    a0, 0
cmp_leave:
  clc   cs0,   (csp)
  clc   cs1,  8(csp)
  cincoffset csp, csp, 16
  cret


  .globl hyperram_cleaner
  .p2align 5
  .type hyperram_cleaner,@function

// Utility function to clean any buffered read data out of the HyperRAM controller interface.
//
// entry  ca0 -> start of an unused memory area in the HyperRAM
//        ca1 -> end of the unused memory area, exclusive
hyperram_cleaner:
  bgeu  a0, a1, clean_leave
  clw   a2, (ca0)
  cincoffset ca0, ca0, 64  // Should be larger than or equal to the burst length
  j     hyperram_cleaner
clean_leave:
  cret


  .globl icache_enabled_set
  .p2align 5
  .type icache_enabled_set,@function

// Set the enabled/disabled state of the icache.
//
// a0 = 1 to enable, 0 to disable.
icache_enabled_set:
  andi  a0, a0, 1
  csrw  0x7c0, a0
  cret

