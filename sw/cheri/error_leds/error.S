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
	// The memory root for storing and loading.
	cspecialr ca0, mtdc
	// The execution root.
	auipcc ca1, 0
	// The sealing root.
	cspecialr ca2, mscratchc

	// Install trap handler.
	la_abs t0, trap
	csetaddr ct0, ca1, t0
	cspecialw mtcc, ct0
	// Enable trap on permit store local capability violation.
	li t0, 1
	csrw 0xBC4, t0
	// Keeps track of which violation to cause next.
	// Start at 8 because first thing we do is add one and wrap to zero.
	li s0, 8

.section .text

nextviolation:
	addi s0, s0, 1 // Go to next exception.
	addi t0, s0, -9 // Check if we hit 9.
	bnez t0, checkvalues
	li s0, 0 // Wrap to zero if we hit 9.
checkvalues:
	li t0, 0
	beq s0, t0, bounds
	addi t0, t0, 1
	beq s0, t0, tag
	addi t0, t0, 1
	beq s0, t0, seal
	addi t0, t0, 1
	beq s0, t0, permitexecute
	addi t0, t0, 1
	beq s0, t0, permitload
	addi t0, t0, 1
	beq s0, t0, permitstore
	addi t0, t0, 1
	beq s0, t0, permitstorecap
	addi t0, t0, 1
	beq s0, t0, permitstoreloccap
	addi t0, t0, 1
	beq s0, t0, permitaccsysreg
	j fail // We should never get here.

bounds:
	// Cause an out of bounds violation.
	csetboundsimm ct0, ca0, 4 // Set bounds to 4.
	clb t0, 5(ct0) // 5 is beyond the bound of 4.
	j fail // We should never get here.

tag:
	// Cause a tag violation
	ccleartag ct0, ca0 // Clear tag.
	clw t0, 0(ct0) // Load from this capability.
	j fail // We should never get here.

seal:
	// Seal a capability.
	li t0, 9 // Used as otype.
	csetaddr ct1, ca2, t0 // Address of ct1 is used as otype.
	cseal ct0, ca0, ct1 // Seal ct1 with sealing root in ca0.
	clw t0, 0(ct0) // Try loading from a sealed capability.
	j fail // We should never get here.

permitexecute:
	// Load the program counter, remove execute permission and try to jumpt to it.
	auipcc ct0, 0
	li t1, 0
	candperm ct0, ct0, t1 // Zero out the permissions (including execute).
	cjalr ct0 // Jump to capability without execute permission.
	j fail // We should never get here.

permitload:
	// Loading from the seal root causes load permission violation.
	clw t0, 0(ca2)
	j fail // We should never get here.

permitstore:
	// Storing to execute root causes store permission violation.
	csw t0, 0(ca1)
	j fail // We should never get here.

permitstorecap:
	// Create data capability with store data permission set.
	li t0, 4
	candperm ct0, ca0, t0
	// Cause store capability permission violation.
	csc ct0, 0(ct0)
	j fail // We should never get here.

permitstoreloccap:
	// Create capability write only capability that is local.
	li t0, 0x44
	candperm ct0, ca0, t0
	// Cause store local capability permission violation.
	csc ct0, 0(ct0)
	j fail // We should never get here.

permitaccsysreg:
	// Remove access system register permission from PCC,
	// as well as permissions: GL, LM, LG, EX, LD, MC
	li t0, 0x16B
	candperm ct0, ca1, t0
	la_abs t1, accesssystemregister
	csetaddr ct0, ct0, t1
	cjalr ct0
accesssystemregister:
	// Accessing a special capability register should trigger the exception.
	cspecialr ct0, mscratchc
	j fail // We should never get here.

// Trap handler must be 4 byte aligned.
.section .text.trap, "ax", @progbits
.p2align 2
trap:
	li t0, 0x01000000 //0x10 for simulation 0x01000000 for FPGA.
delayloop: // Delay clearing error LEDs so that you can see them.
	addi t0, t0, -1
	bnez t0, delayloop
	csrw mtval, 0 // Clear mtval to clear error LED.
	csrw mcause, 0
	j nextviolation // After clearing jump to next exception.

// If we get here we know something went wrong.
fail:
	j fail
