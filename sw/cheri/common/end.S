.include "assembly-helpers.s"

	.section .text, "ax", @progbits
.zero 0x80
	.globl end
	.p2align 2
    .type end,@function
end:
    nop
    nop
    nop
    nop
    nop
