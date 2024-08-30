	.section .text.start, "ax", @progbits
.zero 0x80

	.globl start
	.p2align 2
    .type start,@function
start:
  auipcc           ct1, 0
  li               t0, 0x40000000
  csetaddr         ct1, ct1, t0
  cjr              ct1
