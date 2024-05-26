	.section .text.start, "ax", @progbits
.zero 0x80

	.globl start
	.p2align 2
    .type start,@function
start:
	auipcc           cra, 0
	li               t0, 0x00101000
	csetaddr         cra, cra, t0
	cjr              cra
