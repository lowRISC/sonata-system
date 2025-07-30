# Reference Guide for Randomised Testing using Capability Modification Instructions
## 📚 Table of Contents

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
## CClearTag
This instruction clears the tag of a capability and there isn't a way of testing the instruction that doesn't clear the
tag. 
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
## CUnseal
