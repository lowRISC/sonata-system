.include "assembly-helpers.s"

	.section .text, "ax", @progbits

	.globl get_hyperram_fn_ptr
	.p2align 2
    .type get_hyperram_fn_ptr,@function
get_hyperram_fn_ptr:
  auipcc ct0, 0
  csetaddr ca0, ct0, a0
  cret
