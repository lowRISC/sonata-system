# Reference Guide for Randomised Testing using Capability Modification Instructions
## Contents

- [CAndPerm](#candperm)
- [CClearTag](#ccleartag)
- [CIncAddr](#cincaddr)
- [CIncAddrImm](#cincaddrimm)
- [CSeal](#cseal)
- [CSetAddr](#csetaddr)
- [CSetBounds](#csetbounds)
- [CSetBoundsExact](#csetboundsexact)
- [CSetBoundsRoundDown](#csetboundsrounddown)
- [CSetBoundsImm](#csetboundsimm)
- [CSetHigh](#csethigh)
- [CUnseal](#cunseal)
## CAndPerm
The `CAndPerm` instruction performs a bitwise AND between the permission bits of a capability `cap` and a 32-bit integer
`rand`. If the resulting permissions cannot be represented (by the permission encoding) the result will be a subset
of the ANDed permissions. Different behaviour occurs if `cap` is sealed: if `rand` codes for clearing any permission apart 
from PERMIT_GLOBAL then the tag bit of the result is cleared. This test checks if `cap` is sealed. If it is the test sets
every bit of `rand` apart from the bit associated with PERMIT_GLOBAL, ensuring the tag bit of `cap` will not be cleared. 
### Pseudocode
```python
if cap.type: // Cap is sealed
    rand |= (not 1) // Setting every bit of rs2 apart from the global bit 
CAndPerm(cap, rand)
```
### Assembly
Requires
- Random 32-bit word in a0 
- Arbitrary capability in ca0
```asm
cgettype a1, ca0 // a1 = ca0.type
beqz a1, 1f       // if ca0 is unsealed then go to candperm instruction

// Set all bits of a2 to 1 apart from the global bit
ori a0, a0, -2 // Note -2 = 0xFFFFFFFE

1:
candperm ca0, ca0, a0
```
## CClearTag
This instruction clears the tag of a capability.
## CIncAddr
The `CIncAddr` instruction increments the address of a capability `cap` by a signed integer. To ensure that the resulting address 
remains with the bounds of the capability, the test firstly calculates the maximum value by which the address of `cap` can be incremented and remain valid. 
This is $cap.top - cap.addr$. A random integer `rand` is then used to generate an integer that is below the maximum value 
by which the capability's address can be safely incremented.
### Pseudocode
```
max_increment = cap.top - cap.addr          // The maximum value by which the address can be incremented 
if max_increment > 0:                       // if max_increment is 0 then no increment is possible and the instruction is not run
    increment = rand % max_increment        // Use rand to generate a random number below max_increment 
    cap.addr += increment                   // run CIncAddr
```
### Assembly
Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0

```asm
cgettop a1, ca0 # a1 = ca0.top
cgetaddr a2, ca0 # a2 = ca0.addr
sub a1, a1, a2 # a1 = ca0.top - ca0.addr  // a1 = max_increment
beqz a1, end                              // if max_increment = 0 skip the rest
remu a0 a0, a1 # a0 = a0 % a1             // increment = rand % max_increment
cincoffset ca0, ca0, a0 # ca0.addr += a0  // cap.addr += increment
end:
```
## CIncAddrImm
The `CIncAddrImm` instruction increments the address of a capability `cap` by a signed immediate `IMM`. To ensure that the resulting address
remains with the bounds of the capability, the test firstly calculates the maximum value by which the address of `cap` can be incremented and remain valid.
This is $cap.top - cap.addr$. If `IMM` is below this `CIncAddrImm` is run, otherwise it is skipped. 
### Pseudocode
```python
if cap.base <= cap.addr + imm < cap.top:
    cap.addr += imm
```
### Assembly
Requires
- Random immediate `IMM`
- Arbitrary capability in ca0
```asm
li a0, IMM                      // a0 = imm
cgetaddr a1, ca0                // a1 = cap.addr
add a1, a1, a0                  // a1 = cap.addr + imm
cgettop a2, ca0                 // a2 = cap.top
bgeu a1, a2, 1                  // if cap.addr + imm > top: goto end
cgetbase a2, ca0                // a2 = cap.base
bgtu a2, a1, 1                  // if cap.base > cap.addr + imm: goto end
cincoffsetimm ca0, ca0, IMM     // cap.addr += IMM
1:
```
## CSeal
The `CSeal` instruction seals a capability `cap` using another capability `sealing_cap`. For the operation to be successful,
`sealing_cap` must have `PERMIT_SEAL` and it must be able to set its address to $[0,7]$ or $[9,15]$ depending on whether the capability in `cap` is  executable. 
If `cap` is executable then the address of `sealing_cap` must be set to $[0, 7]$ and otherwise the address of `sealing_cap` must be set to $[9, 15]$. 
In this test, `rand` is used to set the address of `sealing_cap`, which determines the `otype` of `cap` when sealed.
### Pseudocode
```python    
// Check that the capability that is to be used for sealing can set its address to a valid otype (0-7 or 9-16)
if cap.PERMIT_EXECUTE:
    // check that some address in the range [0, 7] is is the bounds of sealing_cap
    if cap.base <= 7 and cap.top > 1:
        cap.addr = (rand % 8)
        cap.addr = min(max(cap.base, cap.addr), cap.top - 1)
    else: 
        return
else:
    // check that some address in the range [9, 15] is in the bounds of sealing_cap
    if cap.base <= 15 & cap.top > 9: //
        cap.addr = 9 + (rand % 7)
        cap.addr = min(max(cap.base, cap.addr), cap.top - 1)
    else:
        return

CSEAL(cap, cap, sealing_cap)     
```
Requires
- Random 32-bit word in `a0`
- Unsealed capability in `ca0`
- Capability with PERMIT_SEAL in `ca1`
### Assembly
```asm
  // Main body
  cgetperm a3, ca0           // a3 = ca0.perms

  li a4, 0x100               // a4 = 2^8

  and a3, a3, a4             // a3 = a3 & a4
  // If a3 is 0 then the executable permission is not set and if a3 is not zero then the executable permission is set

  beqz a3, 1f

  // The execute permission is set. The address of ca1 should be set between 1 and 7
  li t0, 6
  remu a2, a2, t0
  addi a2, a2, 1

  // Now check that the base is below 7 and the top is above 1
  li t0, 8
  cgetbase t1, ca1
  bgeu t1, t0, 3f

  li t0, 1
  cgettop t1, ca1
  bgeu t0, t1, 3f

  // This now needs to be constrained within the bounds of the capability
  cgettop t0, ca1
  bgeu a2, t0, 4f
  j 5f
4:
  mv a2, t0
5:

  cgetbase t0, ca1
  bgeu t0, a2, 6f
  j 7f
6:
  mv a2, t0
7:

  // The address can now be set
  csetaddr ca1, ca1, a2
  j 2f

  // The execute permission is not set
1:
  li t0, 7
  remu a2, a2, t0
  addi a2, a2, 9

  // Now check that the base is below 15 and the top is above 9
  li t0, 16
  cgetbase t1, ca1
  bgeu t1, t0, 3f

  li t0, 9
  cgettop t1, ca1
  bgeu t0, t1, 3f

  // This now needs to be constrained within the bounds of the capability
  cgettop t0, ca1
  bgeu a2, t0, 8f
  j 9f
8:
  mv a2, t0
9:

  cgetbase t0, ca1
  bgeu t0, a2, 10f
  j 11f
10:
  mv a2, t0
11:
  csetaddr ca1, ca1, a2

  // Seal
      
2:
  cseal ca0, ca0, ca1

3:

```
## CSetAddr
The `CSetAddr` instruction sets the address of a capability `cap` to an integer value. 
The test uses the random integer `rand` to generate an integer between `cap.base` and `cap.top` (ensuring `cap` remains valid)
which is used in the `CSetAddr` instruction. 
### Pseudocode
```python
new_address = cap.base + (rand % cap.range)
ca0.addr = new_address
```
### Assembly
Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0
```asm
cgetlen a1, ca0          // a0 = ca0.len
beqz a1, end             // if ca0.len == 0: end
remu a0, a0, a1          // a0 = rand % ca0.len
cgetbase a1, ca0         // a1 = ca0.base
add a0, a0, a1           // a0 = ca0.base + (rand % ca0.len)
csetaddr ca0, ca0, a0    // ca0.addr = ca0.base + (rand % ca0.len)
end: 
```
## CSetBounds
CSetBounds sets new bounds for a capability `cap`, using an integer value corresponding to the new length as input. 
The resulting capability is set to `cap` with 
- its base field replaced with `cap.address`
- its length field replaced with the integer input.

This test ensures that `cap.address` is within the bounds of `cap` and that the integer input value to `CSetBounds` does 
not exceed the maximum permissible length of `cap`. 
By incrementing `cap.addr` by `rand_base % max_increment` (where `rand_base` is a random input to the test) the new address is guaranteed to be below `cap.top`.
A similar technique ensures that the integer input is less than or equal to the maximum permissible length (`cap.top - cap.addr`).

The resulting bounds are not always the same as the requested bounds, but if the requested bounds are a subset of the possible
bounds (bounds of `cap`) the resulting bounds are guaranteed to be at most the bounds of `cap`. For this reason the bounds
check is done on the requested bounds rather than the resulting bounds. This test guarantees that the requested bounds are 
a subset of the bounds of `cap` and so the `CSetBounds` instruction always produces a valid capability (assuming `cap` is
valid and unsealed). 
### Pseudocode
```python
max_increment = cap.top - cap.addr      // Maximum value by which the address can be incremented
if max_increment > 0:                   // if this value is 0, skip
    increment = rand_base % max_increment       // use the random word to produce a random increment
    cap.addr += increment           // Increment the address by a random value
max_increment = cap.top - cap.addr      // Calculate the new maximum length of the capability
if max_increment > 0:
    length = rand_length % max_increment
    new_cap = set_bounds(cap, top)
```
### Assembly
Requires
- Random 32-bit word in a0
- Random 32-bit word in a1
- Arbitrary capability in ca0
```asm
cgettop a2, ca0 # a2 = ca0.top
cgetaddr a3, ca0 # a3 = ca0.addr
sub a2, a2, a3 # a2 = ca0.top - ca0.addr
beqz a2 end
remu a0 a0, a2 # a0 = a0 % a2
cincoffset ca0, ca0, a0 # ca0.addr += a0
cgettop a2, ca0 # a2 = ca0.top
cgetaddr a3, ca0 # a3 = ca0.addr
sub a2, a2, a3 # a2 = ca0.top - ca0.addr
beqz a2 end
remu a0 a1, a2 # a0 = a1 % a2
csetbounds ca0, ca0, a0
end:
```
## CSetBoundsExact
`CSetBoundsExact` sets precise new bounds for a capability `cap`, using an integer value corresponding to the new length as input.
Unlike `CSetBounds`, this instruction will fail (clear the tag bit), if the requested bounds cannot be represented exactly due 
to coding constraints. 
The resulting capability is set to `cap` with its 
- base field replaced with `cap.address`
- length field replaced with the integer input.

This test ensures that `cap.address` is within the bounds of `cap` and that the integer input value to `CSetBounds` does
not exceed the maximum permissible length of `cap`.
By incrementing `cap.addr` by `rand_base % max_increment` (where `rand_base` is a random input to the test) the new address is guaranteed to be below `cap.top`.
A similar technique ensures that the integer input is less than or equal to the maximum permissible length (`cap.top - cap.addr`).

The resulting bounds are always the same as the requested bounds. When this is not representable, the tag bit is cleared. 

The test takes two random 32-bit word inputs, called `rand_base` and `rand_length`, used to determine
the base and length of the resulting capability `cd` respectively.  The address of `cd` is calculated as $cap.address + (rand\_base \bmod cap.length)$.
From this the value of the exponent `e` that will be used to represent the bounds internally is then calculated as the number
of trailing zeroes of this address. `e` cannot be greater than the number of trailing zeroes as the encoding of `base` will then
not be exact (exactness is required for this instruction). `e` could be less than the number of trailing zeroes and this 
case is accounted for when `rand_length` is sufficiently small. 
<!-- This could be improved. If rand_length is small enough then it does not have to align. What should happen is 
before zeroing the bottom bits of rand_length the top bits should be zeroed and a new e calculated from the length. The min
should then be taken with both es can this should be used when zeroing the bottom bits. This is a small change that won't add 
many test cases.-->
### Pseudocode
```python
max_increment = cap.top - cap.addr
if max_increment > 0:
    increment = rand_base % max_increment
    cap.addr += increment
e = clz(cap.addr) 
max_length_bounds = cap.top - cap.addr
if max_length_bounds > 0:
    increment = rand_length % max_length_bounds 
    max_length_alignment = 2 ^ (e + 9)
    min_length_interval = 2^e
    increment -= (increment % max_length_alignment)
    aligned_increment = increment - (increment % min_length_interval)
    csetboundsexact(cap, aligned_increment) 
```
### Assembly
Requires:
- Random 32-bit word in `a0`
- Random 32-bit word in `a1`
- Arbitrary capability in `ca0`
```asm
cgettop a2, ca0            // a2 = ca0.top()
cgetaddr a3, ca0           // a3 = ca0.addr()
sub a2, a2, a3             // a2 = ca0.top() - ca0.addr()
beqz a2, 1f                // if a2 == 0 end 
remu a0, a0, a2            // a0 = rand_base % a2
cincoffset ca0, ca0, a0    // ca0.addr += rand % (ca0.top - ca0.addr)
cgettop a2, ca0            // a2 = ca0.top()
cgetaddr a3, ca0           // a3 = ca0.addr()
sub a2, a2, a3             // a2 = ca0.base() + ca0.bounds() - ca0.addr()
beqz a2, 1f                // if a2 == 0 end
remu a1, a1, a2            // a1 = rand_length % a2
clz a2, a3                 // a2 = clz ca0.addr + rand % (ca0.top - ca0.addr)
li t0, 1                   // t0 = 1
sll t0, t0, a2             // t0 = 1 << a2
addi t0, t0, -1            // t0 = (1 << a2) - 1
not t0, t0
and a1, a1, t0
li t0, 1                   // t0 = 1
sll t0, t0, a2             // t0 = 1 << a2
slli t0, t0, 9
addi t0, t0, -1            // t0 = (1 << a2) - 1
and a1, a1, t0
csetboundsexact ca0, ca0, a1  // ca0.bounds = rand % (ca0.base() + ca0.bounds() - ca0.addr())
1:
```
## CSetBoundsRoundDown
CSetBoundsRoundDown can be tested in exactly the same way as CSetBounds but will have slightly different behaviour (rounding 
the bounds down instead of up).
### Pseudocode
```python
max_increment = cap.top - cap.addr      // Maximum value by which the address can be incremented
if max_increment > 0:                   // if this value is 0, skip
    increment = rand_base % max_increment       // use the random word to produce a random increment
    cap.addr += increment           // Increment the address by a random value
max_increment = cap.top - cap.addr      // Calculate the new maximum length of the capability
if max_increment > 0:
    length = rand_length % max_increment
    new_cap = set_bounds(cap, top)
```
### Assembly
Requires:
- Random 32-bit word in `a0`
- Random 32-bit word in `a1`
- Arbitrary capability in `ca0`
```asm
cgettop a2, ca0 # a2 = ca0.top
cgetaddr a3, ca0 # a3 = ca0.addr
sub a2, a2, a3 # a2 = ca0.top - ca0.addr
beqz a2 end
remu a0 a0, a2 # a0 = a0 % a2
cincoffset ca0, ca0, a0 # ca0.addr += a0
cgettop a2, ca0 # a2 = ca0.top
cgetaddr a3, ca0 # a3 = ca0.addr
sub a2, a2, a3 # a2 = ca0.top - ca0.addr
beqz a2 end
remu a0 a1, a2 # a0 = a1 % a2
csetboundsrounddown ca0, ca0, a0
end:
```
## CSetBoundsImm
CSetBoundsImm takes a capability `cap` and an unsigned immediate `IMM`. The base of the result is set to `cap.addr` and
the length to `IMM`. This test checks whether the new bounds are within the bounds of `cap` and if so executes the instruction. 
### Pseudocode
```python
if cap.addr + IMM < cap.top:
    CSetBoundsImm(cap, IMM)
```
### Assembly
Requires
- Random immediate `IMM`
- Capability in `ca0`
```asm
cgetaddr a3, ca0         // a3 = cap.addr
addi a2, a3, IMM         // a3 = cap.addr + imm
cgettop a4, ca0          // a4 = cap.top
bgeu a2, a4, 1f          // if cap.addr + imm > top: goto end
cgetbase a4, ca0         // a4 = cap.base
bgtu a4, a3, 1f          // if cap.base > cap.addr: goto end
csetboundsimm ca0, ca0, IMM
1:
```
## CSetHigh
While `CSetHigh` always clears the tag bit, it can be used in conjunction with `CBuildCap` to build new tagged
capabilities. As this instruction always clears the tag bit, the test just runs the instruction (knowing that the tag bit
will be cleared).
### Pseudocode
```python
CSetHigh(cap, rand)
```
### Assembly
Requires
- Arbitrary capability in `ca0`
- Random word in `a0`
```asm
csethigh ca0, ca0, a0
```
## CUnseal
CUnseal takes two capabilities, one that is sealed `cap` and that is used to unseal it `unseal_cap`. This test checks that
the otype of `cap` is within the bounds of `unseal_cap` and if this is the case, `unseal_cap` is used to unseal `cap`. There
is no element of randomness in this test, as sealing is a binary operation that is or is not possible.
### Pseudocode
```python
if unseal_cap.base <= cap.type < unseal_cap.top:
    CUnseal(cap, cap, unseal_cap) 
```
### Assembly
Requires
- Sealed capability in `ca0`
- Capability with `PERMIT_UNSEAL` permission in `ca1`
```asm
// Check that the otype of cap is within the bounds on unseal_cap
cgettype a2, ca0
cgetbase a3, ca1
// if ca0.type < ca1.base end
bgeu a3, a2, 1f

cgettop a3, ca1
bgeu a2, a3, 1f

// The otype of cap is within the bounds on unseal_cap

cunseal ca0, ca0, ca1

// Moving values out of the registers
1:
```