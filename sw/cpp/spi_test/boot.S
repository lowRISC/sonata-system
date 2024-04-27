.include "assembly-helpers.s"

	.section .text, "ax", @progbits
.zero 0x80
	.globl start
	.p2align 2
    .type start,@function
start:
	// ca0 (first argument) contains the read-write root
	cspecialr        ca0, mtdc

	// Zero the tag memory
	//li               a1, 0x200fe000
	//csetaddr         ca0, ca0, a1
	//li               a1, 0x20100000
	//cjal             zero_memory

	// No bounds on stack, grows down from the end of IRAM
	li               sp, 0x0010E000
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

	// Zero all of the memory that we haven't loaded into.
	//cspecialr        ca1, mtdc
	//csetaddr         ca0, ca1, a0
	//li               a1, 0x20080000
	//cjal             zero_memory

	// Jump to the newly loaded binary.
	// This could be a relative jump, but I'd need to get the relocations right
	// and we have 32 KiB of IROM so wasting a few bytes doesn't really matter.
	auipcc           cra, 0
	li               t0, 0x00140000
	csetaddr         cra, cra, t0
	cjr              cra

zero_memory:
	csw              zero, 0(ca0)
	cincoffset       ca0, ca0, 4
	blt              a0, a1, zero_memory;
	cret
