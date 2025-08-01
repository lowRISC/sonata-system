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
This instruction takes a capability `cap` and a random integer in `rs2`. It then 'ands' the capability permissions with 
the integer. If the resulting permissions cannot be represented (by the permission encoding) the result will be a subset
of the ANDed permissions. If `cap` is sealed and `rs2` codes for clearing any permission apart from PERMIT_GLOBAL then the 
tag bit of the result is cleared. This test checks if `cap` is sealed, and if so sets every bit of `rs2` apart from the 
bit associated with the global permsission. 
### Pseudocode
```python
rs2 = rand()
if cap.type: // Cap is sealed
    rs2 |= (not 1) // Setting every bit of rs2 apart from the global bit 
CAndPerm(cap, rs2)
```
### Assembly
```asm
cgettype a1, ca0 // a1 = ca0.type
beqz a1, 1       // if ca0 is unsealed then go to candperm instruction

// Set all bits of a2 to 1 apart from the global bit
ori a0, a0, -2 // Note -2 = 0xFFFFFFFE

1:
candperm ca0, ca0, a0
```
## CClearTag
This instruction clears the tag of a capability.
## CIncAddr
CIncOffset is an alias for this instruction.

There are three different cases for the result of this instruction
- Capability valid and address accessible using capability
- Capability valid but address not accessible using capability
- Capability invalid

Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0

### Capability valid and address accessible using capability:
### Explanation 
### Pseudocode
```
max_increment = cap.top - cap.addr
if max_increment > 0:
    increment = rand_val % max_increment
    cap.addr += increment 
```
### Assembly
```asm
cgettop a1, ca0 # a1 = ca0.top
cgetaddr a2, ca0 # a2 = ca0.addr
sub a1, a1, a2 # a1 = ca0.top - ca0.addr
beqz a1, end
remu a0 a0, a1 # a0 = a0 % a1
cincoffset ca0, ca0, a0 # ca0.addr += a0
end:
```
This test guarantees that the address of the new capability is within the bounds of the old capability.
### Capability valid and address not accessible using capability
### Capability invalid 
## CIncAddrImm
Requires
- Random immediate `imm`
- Arbitrary capability in ca0
```asm
li a0, imm # a0 = imm
cgetlen a1, ca0 # a1 = ca0.bounds()
cgetaddr a2, ca0 # a2 = ca0.addr()
cgetbase a3, ca0 # a3 = ca0.base()
sub a2, a2, a3 # a2 = a2 - a3
sub a1, a1, a2 # a1 = a1 - a2 
remu a0 a0, a1 # a0 = a0 % a1
cincoffset ca0, ca0, a0 # ca0.addr += a0
```
## CSeal
This test takes two capabilities. For the test to seal `ca0`, `ca1` must have `PERMIT_SEAL` and it must be able
to set its address to $[0,7]$ or $[9,15]$ depending on whether the capability in `ca0` is  executable. If `ca0` is executable
then the address of `ca1` must be set to $[0, 7]$ and otherwise the address of `ca1` must be set to $[9, 15]$. 

Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0, `cap`
- Capability with PERMIT_SEAL in ca1, `sealing_cap`
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
Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0
### Explanation
### Pseudocode
```python
new_address = ca0.base + (rand % ca0.range)
ca0.addr = new_address
```
### Assembly
```asm
cgetlen a1, ca0       # a0 = ca0.len
beqz a1, end          # if ca0.len == 0: end
remu a0, a0, a1       # a0 = rand % ca0.len
cgetbase a1, ca0      # a1 = ca0.base
add a0, a0, a1        # a0 = ca0.base + (rand % ca0.len)
csetaddr ca0, ca0, a0 # ca0.addr = ca0.base + (rand % ca0.len)
end: 
```
## CSetBounds
Requires
- Random 32-bit word in a0
- Random 32-bit word in a1
- Arbitrary capability in ca0
### Explanation 
CSetBounds takes a destination capability register `cd`, a source capability register `cs1` and an integer register `rs2`. The 
destination capability is set to the source capability with its base field replaced with `cs1.address` and its length field
replaced with the integer register `rs2`. 

This test guarantees that the cs1.address is within the bounds of cs1 and further guarantees that the value in rs2 does
not exceed the maximum possible length of the capability (cd.top cannot be greater than cs1.top). By incrementing `cap.addr`
by `rand_base % max_increment` (where `rand_base` is a random input) the new address is guaranteed to be below `cap.top`.
The `length` to be used in set bounds is also less than or equal to the maximum possible length by once again calculating 
the difference between the current address and the top (`max_increment = cap.top - cap.addr`) and ensuring that the length 
is less than or equal to this value we guarantee that the bounds of `cd` cannot exceed the bounds of `cs1`. 

The resulting bounds are not always the same as the requested bounds, but if the requested bounds are less than the possible
bounds (bounds of `cs1`) the resulting bounds are guaranteed to be at most the bounds of `cs1`. For this reason the bounds
check is done on the requested bounds rather than the resulting bounds. This test guarantees that the requested bounds are 
a subset of the bounds of `cs1` and so the `CSetBounds` instruction always produces a valid capability (assuming `cs1` is
valid and unsealed). 
### Pseudocode
```python
rand_base = random_int()
rand_length = random_int()
max_increment = cap.top - cap.addr
if max_increment > 0:
    increment = rand_base % max_increment
    cap.addr += increment
max_increment = cap.top - cap.addr
if max_increment > 0:
    length = rand_length % max_increment
    new_cap = set_bounds(cap, top)
```
### Assembly
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
CSetBoundsExact takes a destination capability register `cd`, a source capability register and an integer register `rs2`.
Capability register `cd` is set to `cs1` with its base (and address) as `cs1.address` and its length as `rs2`. If this cannot be represented
exactly the tag bit is cleared. Like with CSetBounds, if the requested bounds for `cd` are larger than the bounds for `cs1`
the tag bit is cleared.

For the bounds to be represented exactly, the number of leading zeroes of the length must be greater than or equal to the
number of trailing zeroes of the base. The length of the capability `cd` is determined by the integer register `rs2` and the 
base of `cd` is determined by `cs1.address`. There are many different ways to manipulate random inputs for this test to ensure the instruction succeeds.

The following test takes two random 32-bit word inputs, called `rand_base` and `rand_length`, because they are used to determine
the base and length of `cd` respectively.  The address of `cd` is calculated as $cs1.address + (rand\_base \bmod cs1.length)$.
From this the value of the exponent `e` that will be used to represent the bounds internally is then calculated as the number
of trailing zeroes of this address. `e` cannot be greater than the number of trailing zeroes as the encoding of `base` will then
not be exact (exactness is required for this instruction). `e` could be less than the number of trailing zeroes and this 
case is accounted for when `rand_length` is sufficiently small. 
<!-- This could be improved. If rand_length is small enough then it does not have to align. What should happen is 
before zeroing the bottom bits of rand_length the top bits should be zeroed and a new e calculated from the length. The min
should then be taken with both es can this should be used when zeroing the bottom bits. This is a small change that won't add 
many test cases.-->

Requires:
- Random 32-bit word in a0, `rand_base`
- Random 32-bit word in a1, `rand_length`
- Arbitrary capability in ca0
### Pseudocode
```python
rand_base = random_int()
rand_length = random_int()
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
1:\n"
```
## CSetBoundsRoundDown
CSetBoundsRoundDown can be tested in exactly the same way as CSetBounds but will have slightly different behaviour (rounding 
the bounds down instead of up).
### Pseudocode
### Assembly
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
## CSetHigh
Always clears the tag bit 
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