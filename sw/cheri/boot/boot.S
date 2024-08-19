.include "assembly-helpers.s"

	.section .text.start, "ax", @progbits
.zero 0x80

	.globl start
	.p2align 2
    .type start,@function
start:
	// ca0 (first argument) contains the read-write root
	cspecialr        ca0, mtdc

	// No bounds on stack, grows down from the end of IRAM
	li               sp, 0x00100ff0
	csetaddr         csp, ca0, sp
	auipcc           cra, 0
	// Call the C++ entry point
	la_abs           t0, rom_loader_entry
	csetaddr         cra, cra, t0
	// Store capability to stack
	csc              cra, 0(csp)
	// Load capability from stack
	clc              cra, 0(csp)
	// Clear capability from stack
	csc              cnull, 0(csp)
	cjalr            cra

	// Jump to the newly loaded binary.
	// This could be a relative jump, but I'd need to get the relocations right
	// and we have 32 KiB of IROM so wasting a few bytes doesn't really matter.
	auipcc           cra, 0
	li               t0, 0x00101000
	csetaddr         ct0, cra, t0
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
