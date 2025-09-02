# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

// A set of memory initialisation functions using different access sizes, to check that
// the write transactions are properly coalesced into write bursts to the HyperBus
// Memory Controller.
//
// Each routine is expected to be writing a single, defined byte to each address that is
// modified. By initialising the entirety of a target buffer to a different value first,
// the set of modified addresses may be ascertained.

  .section .text, "ax", @progbits

  .option norvc

// Byte-based memory writing.
//
// entry ca0 -> byte-aligned destination buffer
//       a1   = byte to be stored
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_b
  .p2align 5
hyperram_memset_b:
  addi  a2, a2, -8
  bltz  a2, memset_b_8fix
memset_b_8:
  csb   a1,  (ca0)
  csb   a1, 1(ca0)
  csb   a1, 2(ca0)
  csb   a1, 3(ca0)
  csb   a1, 4(ca0)
  csb   a1, 5(ca0)
  csb   a1, 6(ca0)
  csb   a1, 7(ca0)
  cincoffset ca0, ca0, 8
  addi  a2, a2, -8
  bgez  a2, memset_b_8
memset_b_8fix:
  addi  a2, a2, 8
  bgtz  a2, memset_b_tail
  cret

// Just complete the request using byte stores; this is shared among all _ascending_ routines.
// Performance is not very important, but it would be better to keep the byte writes temporally
// close together to try to provoke races.
//
// ca0 -> byte-aligned pointer into destination buffer
// a1  =  byte to be stored
// a2  =  non-zero count of bytes remaining
memset_b_tail:
  add   a2, a0, a2
memset_b_1:
  csb   a1, (ca0)
  cincoffset ca0, ca0, 1
  bltu  a0, a2, memset_b_1
  cret

// Descending transfer, pre-decrementing address; shared among the two _descending_ routines.
//
// ca0 -> just beyond next address to be written, decrement before use.
// a1  =  byte to be stored
// a2  =  non-zero count of bytes remaining
memset_b_desc_tail:
  sub   a2, a0, a2
memset_b_desc_1:
  csb   a1, -1(ca0)
  cincoffset ca0, ca0, -1
  bgtu  a0, a2, memset_b_desc_1
  cret

// Byte and half word-based memory writing; each word is written using
// 2 byte stores and a half-word store.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_hb
  .p2align 5
hyperram_memset_hb:
  addi  a2, a2, -4
  bltz  a2, memset_hb_4fix
memset_hb_4:
  csb   a1,  (ca0)
  csb   a1, 1(ca0)
  csh   a1, 2(ca0)
  cincoffset ca0, ca0, 4
  addi  a2, a2, -4
  bgez  a2, memset_hb_4
memset_hb_4fix:
  addi  a2, a2, 4
  bgtz  a2, memset_b_tail
  cret

// Half word-based memory writing.
//
// entry ca0 -> half-word aligned destination buffer
//       a1   = byte to be stored, replicated throughout half-word
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_h
  .p2align 5
hyperram_memset_h:
  addi  a2, a2, -16
  bltz  a2, memset_h_16fix
memset_h_16:
  csh   a1,   (ca0)
  csh   a1,  2(ca0)
  csh   a1,  4(ca0)
  csh   a1,  6(ca0)
  csh   a1,  8(ca0)
  csh   a1, 10(ca0)
  csh   a1, 12(ca0)
  csh   a1, 14(ca0)
  cincoffset ca0, ca0, 16
  addi  a2, a2, -16
  bgez a2, memset_h_16
memset_h_16fix:
  addi  a2, a2, 16
  bgtz  a2, memset_b_tail
  cret

// Word-based memory writing.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_w
  .p2align 5
hyperram_memset_w:
  addi  a2, a2, -32
  bltz  a2, memset_w_32fix
memset_w_32:
  csw   a1,   (ca0)
  csw   a1,  4(ca0)
  csw   a1,  8(ca0)
  csw   a1, 12(ca0)
  csw   a1, 16(ca0)
  csw   a1, 20(ca0)
  csw   a1, 24(ca0)
  csw   a1, 28(ca0)
  cincoffset ca0, ca0, 32
  addi  a2, a2, -32
  bgez  a2, memset_w_32
memset_w_32fix:
  addi  a2, a2, 32
  bgtz  a2, memset_b_tail
  cret

// Repeated words memory writing; the performance of this code is of no consequence.
// It is concerned purely with ensuring the correctness of the written data.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_wr
  .p2align 5
hyperram_memset_wr:
  addi  a2, a2, -4
  bltz  a2, memset_wr_4fix
  xori  a3, a1, -1
memset_wr_4:
  csw   a3, (ca0)  // This word should be overwritten...
  csw   a1, (ca0)  // ...by the original value.
  cincoffset ca0, ca0, 4
  addi  a2, a2, -4
  bgez  a2, memset_wr_4
memset_wr_4fix:
  addi  a2, a2, 4
  bgtz  a2, memset_b_tail
  cret

// Word-based memory writing to descending addresses.
//
// entry ca0 -> word-aligned end of destination buffer, exclusive
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_wd
  .p2align 5
hyperram_memset_wd:
  addi  a2, a2, -8
  bltz  a2, memset_wd_8fix
memset_wd_8:
  csw   a1, -4(ca0)
  csw   a1, -8(ca0)
  cincoffset ca0, ca0, -8
  addi  a2, a2, -8
  bgez  a2, memset_wd_8
memset_wd_8fix:
  addi  a2, a2, 8
  bgtz  a2, memset_b_desc_tail
  cret

// Capability stores to ascending addresses.
//
// These are issued as two back-to-back word writes and we're just using
// this as a way to issue 64-bit writes rather than trying to create
// sensible/valid capabilities.
//
// entry ca0 -> double-word aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_c
  .p2align 5
hyperram_memset_c:
  // Replicate the data word to yield a double word.
  cincoffset csp, csp, -8
  csw   a1,  (csp)
  csw   a1, 4(csp)
  clc   ca1,(csp)
  cincoffset csp, csp, 8
  addi  a2, a2, -8
  bltz  a2, memset_c_8fix
memset_c_8:
  csc   ca1, (ca0)
  cincoffset ca0, ca0, 8
  addi  a2, a2, -8
  bgez  a2, memset_c_8
memset_c_8fix:
  addi  a2, a2, 8
  bgtz  a2, memset_b_tail
  cret

// Capability stores to descending addresses. See above.
//
// entry ca0 -> double-word aligned end of destination buffer, exclusive
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_cd
  .p2align 5
hyperram_memset_cd:
  // Replicate the data word to yield a double word.
  cincoffset csp, csp, -8
  csw   a1,  (csp)
  csw   a1, 4(csp)
  clc   ca1, (csp)
  cincoffset csp, csp, 8
  addi  a2, a2, -8
  bltz  a2, memset_cd_8fix
memset_cd_8:
  csc   ca1, -8(ca0)
  cincoffset ca0, ca0, -8
  addi  a2, a2, -8
  bgez  a2, memset_cd_8
memset_cd_8fix:
  addi  a2, a2, 8
  bgtz  a2, memset_b_desc_tail
  cret

// Utility function returning the `pcc` at which an exception occurred.
  .globl get_mepcc
  .p2align 5
get_mepcc:
  cspecialr ca0, mepcc
  cret
