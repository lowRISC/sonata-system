# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

.include "assembly-helpers.s"

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

	// Install a trap handler to allow diagnostics when things go wrong.
	la_abs           t0, trap
	csetaddr         ct0, cs1, t0
	cspecialw        mtcc, ct0

	// No bounds on stack, grows down from the end of IRAM
	li               sp, 0x00100ff0
	csetaddr         csp, cs0, sp

	// Clear the revocation bitmap before entering C code.
	// The bitmap is not cleared upon reset so memset to return it to a
	// pristine state.
	li               a0, 0x30000000
	csetaddr         ca0, cs0, a0
	li               a1, 0
	li               a2, 4096
	ccall            bl_memset

	cmove            ca0, cs0
	ccall            rom_loader_entry

	// Flush ICache now we've loaded new code
	fence.i

	// Jump to the newly loaded binary.
	csetaddr         ct0, cs1, a0
	cjr              ct0

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

.section .text.trap, "ax", @progbits
// Trap handler must be 4 byte aligned.
.p2align 2
trap:
	cspecialr       ca0, mtdc

	// No bounds on stack, grows down from the end of IRAM
	li              sp, 0x00100ff0
	csetaddr        csp, ca0, sp

	ccall           exception_handler

	// Infinite loop if the exception handler even returns
1:
	wfi
	j 1b
