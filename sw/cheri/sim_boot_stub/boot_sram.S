# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
  .section .text.start, "ax", @progbits
  .zero 0x80

  .globl start
  .p2align 2
  .type start,@function

#define SRAM_BASE 0x00101000

start:
  // Enable the ICache
  csrsi            0x7c0, 1
  auipcc           ct1, 0
  # Jumping to the start of code in SRAM located at 0x00101000 by default.
  li               t0, SRAM_BASE
  csetaddr         ct1, ct1, t0
  cjr              ct1
