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

.section .text
  // Get capability to storage in ct0
  la_abs t0, storage
  srli t0, t0, 3 // Make sure it is 64-bit aligned
  slli t0, t0, 3
  csetaddr ct0, ca0, t0
  // Check that tags start out as 0
  clc ct1,  0(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1,  8(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1, 16(ct0)
  cgettag t2, ct1
  bnez t2, fail
  // Store valid capabilities
  csc ct0,  0(ct0)
  csc ct0,  8(ct0)
  csc ct0, 16(ct0)
  // Check that tags are all valid
  clc ct1,  0(ct0)
  cgettag t2, ct1
  beqz t2, fail
  clc ct1,  8(ct0)
  cgettag t2, ct1
  beqz t2, fail
  clc ct1, 16(ct0)
  cgettag t2, ct1
  beqz t2, fail
  // Invalidate second capability with write to top word
  csw zero, 8(ct0)
  // Check tags are 1,0,1
  clc ct1,  0(ct0)
  cgettag t2, ct1
  beqz t2, fail
  clc ct1,  8(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1, 16(ct0)
  cgettag t2, ct1
  beqz t2, fail
  // Write back valid capability
  csc ct0,  8(ct0)
  // Invalidate second capability with write to middle byte
  csb zero,13(ct0)
  // Check that tags are 1,0,1
  clc ct1,  0(ct0)
  cgettag t2, ct1
  beqz t2, fail
  clc ct1,  8(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1, 16(ct0)
  cgettag t2, ct1
  beqz t2, fail
  // Invalidate other two capabilities
  csh zero, 4(ct0)
  csw zero,20(ct0)
  // Check that tags are all invalid
  clc ct1,  0(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1,  8(ct0)
  cgettag t2, ct1
  bnez t2, fail
  clc ct1, 16(ct0)
  cgettag t2, ct1
  bnez t2, fail

success:
  j success

  nop // In case of misalignment
  nop
  nop
storage:
  // First capability
  nop
  nop
  nop
  nop
  // Second capability
  nop
  nop
  nop
  nop
  // Third capability
  nop
  nop
  nop
  nop

// If we get here we know something went wrong.
fail:
  j fail
