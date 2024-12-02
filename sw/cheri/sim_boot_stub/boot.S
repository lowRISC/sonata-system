# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
  .section .text.start, "ax", @progbits
  .zero 0x80

  .globl start
  .p2align 2
  .type start,@function

#define HYPERRAM_BASE 0x40000000

start:
  // Enable the ICache
  csrsi            0x7c0, 1
  auipcc           ct1, 0
  // Jump to HyperRAM to start executing
  li               t0, HYPERRAM_BASE
  csetaddr         ct1, ct1, t0
  cjr              ct1
