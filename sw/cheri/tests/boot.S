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
	// ca0 (first argument) contains the read-write root
	cspecialr        ca0, mtdc

	// No bounds on stack, grows down from the end
	li               sp, 0x0010E000
	csetaddr         csp, ca0, sp
	auipcc           cra, 0
	// Call the C++ entry point
	la_abs           t0, entry_point
	csetaddr         cra, cra, t0
	// Store capability to stack
	csc              cra, 0(csp)
	// Load capability from stack
	clc              cra, 0(csp)
	// Clear capability from stack
	csc              cnull, 0(csp)
	cjalr            cra
