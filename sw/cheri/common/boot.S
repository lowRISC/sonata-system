# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
.include "assembly-helpers.s"

  .section .vectors, "ax"
  .p2align 4
  .global _trap_vector
  .type _trap_vector, @function
_trap_vector:
  // Save all the register to the stack
  cincoffset csp, csp, -(8 * 15)
  csc   c1,  1 * 8(csp)
  csc   c2,  2 * 8(csp)
  csc   c3,  3 * 8(csp)
  csc   c4,  4 * 8(csp)
  csc   c5,  5 * 8(csp)
  csc   c6,  6 * 8(csp)
  csc   c7,  7 * 8(csp)
  csc   c8,  8 * 8(csp)
  csc   c9,  9 * 8(csp)
  csc   c10,  10 * 8(csp)
  csc   c11,  11 * 8(csp)
  csc   c12,  12 * 8(csp)
  csc   c13,  13 * 8(csp)
  csc   c14,  14 * 8(csp)
  csc   c15,  15 * 8(csp)

  ccall __trap_vector

.L_isr_exit:
  // Restore all registers from the stack.
  clc   c1,  1 * 8(csp)
  clc   c2,  2 * 8(csp)
  clc   c3,  3 * 8(csp)
  clc   c4,  4 * 8(csp)
  clc   c5,  5 * 8(csp)
  clc   c6,  6 * 8(csp)
  clc   c7,  7 * 8(csp)
  clc   c8,  8 * 8(csp)
  clc   c9,  9 * 8(csp)
  clc   c10,  10 * 8(csp)
  clc   c11,  11 * 8(csp)
  clc   c12,  12 * 8(csp)
  clc   c13,  13 * 8(csp)
  clc   c14,  14 * 8(csp)
  clc   c15,  15 * 8(csp)
  cincoffset csp, csp, (8 * 15)
  mret
  // Set size so vector can be disassembled.
  .size _trap_vector, .-_trap_vector


	.section .text.start, "ax", @progbits
.zero 0x80

	.globl start
	.p2align 2
    .type start,@function
start:
	// Enable the ICache
	csrsi            0x7c0, 1

	// Upon reset mtdc contains the read-write root and pcc contains the execution root.
	cspecialr        cs0, mtdc
	auipcc           cs1, 0

	// No bounds on stack, grows down from the end of IRAM
	li               sp, 0x00120000
	csetaddr         csp, cs0, sp

  //Set interrupt/exception handlers.
  la_abs   t0, _trap_vector
  csetaddr ct0, cs1, t0
  cspecialw mtcc, ct0

	// Clear the revocation bitmap before entering C/C++ code.
	// The bitmap is not cleared upon reset so memset to return it to a
	// pristine state.
	li               a0, 0x30000000
	csetaddr         ca0, cs0, a0
	li               a1, 0
	li               a2, 2048
	ccall            bl_memset

	// Set cgp to correct location so globals can be used.
.extern __global_pointer$
	la_abs           t0, __global_pointer$
	csetaddr         cgp, cs0, t0

	cmove            ca0, cs0
	ccall            entry_point

	// Infinite loop if the entry point ever returns
1:
	wfi
	j 1b

.section .text.bl_memset, "ax", @progbits
.global bl_memset
bl_memset:
	// Check if everything is aligned, and if so use word fill.
	andi            a3, a0, 3
	bnez            a3, .Lbytes_fill
	andi            a3, a2, 3
	bnez            a3, .Lbytes_fill

	// Broadcast a1 to all bytes.
	andi            a1, a1, 0xff
	slli            a3, a1, 8
	or              a1, a3, a1
	slli            a3, a1, 16
	or              a1, a3, a1

0:
	beqz            a2, .Lret
	csw             a1, (ca0)
	cincoffset      ca0, ca0, 4
	addi            a2, a2, -4
	j               0b

.Lbytes_fill:
0:
	beqz            a2, .Lret
	csb             a1, (ca0)
	cincoffset      ca0, ca0, 1
	addi            a2, a2, -1
	j               0b

.Lret:
	cret
