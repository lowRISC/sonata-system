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

## Introduction

This reference guide presents tests for each capability modification instruction that yield a tagged and modified output capability.
Using random instruction generation for testing these instructions has been challenging, as random capability modification instructions clear the tag in the majority of cases.
Therefore, the tests in this guide are randomized but constrained to ensure the capability remains tagged.

## CAndPerm

The `CAndPerm` instruction performs a bitwise AND between the permission bits of a capability `cap` and a 32-bit integer.
If the resulting permissions cannot be represented (by the permission encoding) the result will be a subset of the ANDed permissions.
Different behaviour occurs if `cap` is sealed: if the integer input codes for clearing any permission apart from PERMIT_GLOBAL then the tag bit of the result is cleared.
This test checks if `cap` is sealed.
If it is the test sets every bit of its random word input, `rand`, apart from the bit associated with PERMIT_GLOBAL, ensuring the tag bit of `cap` will not be cleared.

### Pseudocode

```
if cap.type: // Cap is sealed
    rand |= (not 1) // Setting every bit of rs2 apart from the global bit
CAndPerm(cap, rand)
```

### Assembly

```asm
// Constrained CAndPerm routine that will maintain the tag bit of a given capability 
// while modifying its permissions using a given random word
//
// ca0    = capability to modify
// a2     = random 32-bit word
// return = modified capability
valid_candperm:
// if given capability is unsealed then go to candperm instruction
cgettype a3, ca0 // a3 = ca0.type
beqz a3, valid_candperm_do
// Set all bits of a2 apart from the global bit
ori a2, a2, -2 // Note -2 = 0xFFFFFFFE

valid_candperm_do:
candperm ca0, ca0, a2
```

## CClearTag

This instruction clears the tag of a capability and so there is no test where the capability's tag is not cleared.

## CIncAddr

The `CIncAddr` instruction increments the address of a capability `cap` by a signed integer.
To ensure that the resulting address remains with the bounds of the capability, the test firstly calculates the difference between the top and base of the capability.
A random integer `rand` is then used to generate a signed integer that, when added to the address, yields an address within the bounds of `cap`.
$$inc = (cap.base - cap.address) + (rand \bmod (cap.top - cap.base))$$
$$cap.address + inc = cap.base + (rand \bmod (cap.top - cap.base))$$
**Note:** The new address is between the base and top of `cap`. This is a subset of the representable range, which goes from the $cap.base$ to $cap.base + 2^{e+9}$ where $e$ is the exponent used to encode the bounds. This choice was made so that the capability can then be used without causing an exception. The same decision has been made for `CIncAddrImm` and `CSetAddr`.
### Pseudocode

```
range = cap.top - cap.base                  // the length of the bounds
if range > 0:                               // if max_increment is 0 then no increment is possible and the instruction is not run
    increment = rand % range        // Use rand to generate a random number below range
    increment -= (cap.addr - cap.base)      // increment = cap.base + (rand % range)
    cap.addr += increment                   // run CIncAddr
```

### Assembly

```asm
// Constrained CIncAddr routine that will maintain the tag bit of a given capability
// while incrementing its address using a given random word
//
// ca0    = capability to modify
// a2     = random 32-bit word
// return = modified capability
valid_cincaddr:
cgettop a3, ca0        
cgetbase a4, ca0       
sub a3, a3, a4         // range = cap.top - cap.base
beqz a4, end            // if cap.top - cap.base is 0, the address cannot be changed and these rest of the test is skipped 
remu a2, a2, a3        // inc = rand % range
cgetaddr a3, ca0       
sub a3, a3, a4         // a3 = cap.addr - cap.base
sub a2, a2, a3         // inc = inc - (cap.addr - cap.base)
cincoffset ca0, ca0, a2
end:
```

## CIncAddrImm

The `CIncAddrImm` instruction increments the address of a capability `cap` by a signed immediate `IMM`.
To ensure that the resulting address remains with the bounds of the capability, the test calculates the maximum and minimum values by which the address of `cap` can be incremented and remain tagged.
The test only runs the `CIncAddrImm` instruction if `IMM` is between these values.

### Pseudocode

```
if cap.base <= cap.addr + imm < cap.top:
    cap.addr += imm
```

### Assembly

```asm
// Constrained CIncAddrImm routine that will maintain the tag bit of a given capability
// while incrementing its address using an immediate IMM
//
// ca0    = capability to modify
// return = modified capability
valid_cincaddrimm:
addi a2, zero, IMM                      
cgetaddr a3, ca0                
add a3, a3, a2                  // a3 = cap.addr + imm
cgettop a4, ca0                 // a4 = cap.top
bgeu a3, a4, end                // if cap.addr + imm > top then instruction will clear tag and skip rest of algorithm 
cgetbase a4, ca0                
bgtu a4, a3, end                // if cap.base > cap.addr + imm then instruction will clear tag and skip
cincoffsetimm ca0, ca0, IMM     // cap.addr += IMM
end:
```

## CSeal

The `CSeal` instruction seals a capability `cap` using another capability `sealing_cap`.
For the operation to be successful, `sealing_cap` must have `PERMIT_SEAL` and it must be able to set its address to $[0,7]$
or $[9,15]$ depending on whether the capability in `cap` is  executable.
If `cap` is executable then the address of `sealing_cap` must be set to $[0, 7]$ and otherwise the address of `sealing_cap` must be set to $[9, 15]$.
In this test, `rand` is used to set the address of `sealing_cap`, which determines the `otype` of `cap` when sealed.

### Pseudocode

```
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
// Constrained CSeal routine that will maintain the tag bit of a given capability
// while sealing the capability, if given another capability with permission to do so
//
// ca0    = unsealed capability to seal
// ca1    = capability with PERMIT_SEAL used to seal capability in ca0
// a2     = random 32-bit word
// return = modified capability
valid_cseal: 
// Main body
cgetperm t0, ca0           // t0 = ca0.perms

li t1, 0x100               // t1 = 2^8

and t0, t0, t1             // t0 = t0 & t1
// If t0 is 0 then the executable permission is not set and if t0 is not zero then the executable permission is set

beqz t0, 1f

// The execute permission is set. The address of ca1 should be set between 1 and 7
li t2, 6
remu a4, a4, t2
addi a4, a4, 1

// Now check that the base is below 7 and the top is above 1
li t2, 8
cgetbase t3, ca1
bgeu t3, t2, 3f

li t2, 1
cgettop t3, ca1
bgeu t2, t3, 3f

// This now needs to be constrained within the bounds of the capability
cgettop t2, ca1
bgeu a4, t2, 4f
j 5f
4:
mv a4, t2
5:

cgetbase t2, ca1
bgeu t2, a4, 6f
j 7f
6:
mv a4, t2
7:

// The address can now be set
csetaddr ca1, ca1, a4
j 2f

// The execute permission is not set
1:
li t2, 7
remu a4, a4, t2
addi a4, a4, 9

// Now check that the base is below 15 and the top is above 9
li t2, 16
cgetbase t3, ca1
bgeu t3, t2, 3f

li t2, 9
cgettop t3, ca1
bgeu t2, t3, 3f

// This now needs to be constrained within the bounds of the capability
cgettop t2, ca1
bgeu a4, t2, 8f
j 9f
8:
mv a4, t2
9:

cgetbase t2, ca1
bgeu t2, a4, 10f
j 11f
10:
mv a4, t2
11:
csetaddr ca1, ca1, a4

// Seal

2:
cseal ca0, ca0, ca1

3:

```

## CSetAddr

The `CSetAddr` instruction sets the address of a capability `cap` to an integer value.
The test uses the random integer `rand` to generate an integer between `cap.base` and `cap.top` (ensuring `cap` remains tagged) which is used in the `CSetAddr` instruction.

### Pseudocode

```
new_address = cap.base + (rand % cap.range)
ca0.addr = new_address
```

### Assembly

```asm
// Constrained CSetAddr routine that will maintain the tag bit of a given capability 
// while setting its address using a given random word
//
// ca0    = capability to modify
// a2     = random 32-bit word
// return = modified capability
valid_csetaddr:
cgetbase a3, ca0        
cgettop a4, ca0         
sub a3, a4, a3          // a3 = ca0.top - ca0.base
beqz a3, end            // if ca0.top - ca0.base == 0 then range is 0 so skip rest of test 
remu a2, a2, a3         
cgetbase a3, ca0        
add a2, a2, a3          // a2 = ca0.base() + ( rand % ca0.length() )
csetaddr ca0, ca0, a2
end:
```

## CSetBounds

CSetBounds sets new bounds for a capability `cap`, using an unsigned integer value corresponding to the new length as input.
The resulting capability is set to `cap` with:
- its base field replaced with `cap.address`
- its length field replaced with the integer input.

This test ensures that `cap.address` is within the bounds of `cap` and that the integer input value to `CSetBounds` does not exceed the maximum permissible length of `cap`.
The test firstly sets `cap.address` to a random value within the bounds.
The test then ensures that the integer input to `CSetBounds` is less than or equal to the maximum permissible length (`cap.top - cap.addr`).

The resulting bounds are not always the same as the requested bounds, but if the requested bounds are a subset of the possible bounds (bounds of `cap`) the resulting bounds are guaranteed to be at most the bounds of `cap`.
For this reason the bounds check is done on the requested bounds rather than the resulting bounds.
This test guarantees that the requested bounds are a subset of the bounds of `cap` and so the `CSetBounds` instruction always produces a tagged capability (assuming `cap` is tagged and unsealed).

### Pseudocode

```
new_address = cap.base + (rand_base % cap.range)
ca0.addr = new_address
max_increment = cap.top - cap.addr      // Calculate the new maximum length of the capability
if max_increment > 0:
    length = rand_length % max_increment
    new_cap = set_bounds(cap, top)
```

### Assembly

```asm
// Constrained CSetBounds routine that will maintain the tag bit of a given capability 
// while setting setting the bounds using two random words
//
// ca0    = capability to modify
// a2     = random 32-bit word
// a3     = random 32-bit word
// return = modified capability
valid_csetbounds:
cgetbase a4, ca0        
cgettop t0, ca0         
sub a4, t0, a4          // a3 = ca0.top - ca0.base
beqz a4, end            // if ca0.top - ca0.base == 0 then cannot shrink bounds so skip rest of test
remu a2, a2, a4         
cgetbase a4, ca0        
add a2, a2, a4          // a2 = ca0.base() + ( rand % ca0.length )
csetaddr ca0, ca0, a2
cgettop a4, ca0 
cgetaddr t0, ca0 
sub a4, a4, t0          // a4 = ca0.top - ca0.addr
beqz a4 end             // if ca0.top - ca0.addr == 0 then cannot shrink bounds so skip rest of test
remu a2 a3, a4          // length = rand_length % (ca0.top - ca0.addr)
csetbounds ca0, ca0, a2
end:
```

## CSetBoundsExact

`CSetBoundsExact` sets precise new bounds for a capability `cap`, using an integer value corresponding to the new length as input.
Unlike `CSetBounds`, this instruction will fail (clear the tag bit), if the requested bounds cannot be represented exactly due to coding constraints.
The resulting capability is set to `cap` with its:
- base field replaced with `cap.address`
- length field replaced with the integer input.

This test ensures that `cap.address` is within the bounds of `cap` and that the integer input value to `CSetBounds` does not exceed the maximum permissible length of `cap`.
The test firstly sets `cap.address` to a random value within the bounds.
The test then ensures that the integer input to `CSetBoundsExact` is less than or equal to the maximum permissible length (`cap.top - cap.addr`).

The resulting bounds are always the same as the requested bounds. When this is not representable, the tag bit is cleared.

The test takes two random 32-bit word inputs, called `rand_base` and `rand_length`, used to determine the base and length of the resulting capability `cd` respectively.
The address of `cd` is calculated as $cap.address + (rand_base \bmod cap.length)$.
From this the value of the exponent `e` that will be used to represent the bounds internally is then calculated as the number of trailing zeroes of this address.
`e` cannot be greater than the number of trailing zeroes as the encoding of `base` will then not be exact (exactness is required for this instruction).
`e` could be less than the number of trailing zeroes and this case is accounted for when `rand_length` is sufficiently small.

### Pseudocode

```
new_address = cap.base + (rand_base % cap.range)
ca0.addr = new_address
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
// Constrained CSetBoundsExact routine that will maintain the tag bit of a given capability 
// while setting setting the bounds using two random words
//
// ca0    = capability to modify
// a2     = random 32-bit word
// a3     = random 32-bit word
// return = modified capability
valid_csetboundsexact:
cgetbase a4, ca0           
cgettop t0, ca0            
sub a4, t0, a4             // a3 = ca0.top - ca0.base
beqz a4, 1f                // if ca0.top - ca0.base == 0 then bounds cannot be constrained so skip rest of test 
remu a2, a2, a4            // inc = rand_base % (ca0.top - ca0.base)
cgetbase a4, ca0           
add a2, a2, a4             
csetaddr ca0, ca0, a2      // ca0.addr = ca0.base + inc
cgettop a4, ca0            
cgetaddr t0, ca0           
sub a4, a4, t0             // a4 = ca0.base() + ca0.bounds() - ca0.addr()
beqz a4, 1f                // if a4 == 0 end
remu a3, a3, a4            // a3 = rand_length % a4
clz a4, t0                 // a4 = clz ca0.addr + rand % (ca0.top - ca0.addr)
li t2, 1                   
sll t2, t2, a4
addi t2, t2, -1            // t2 = (1 << a4) - 1
not t2, t2
and a3, a3, t2
li t2, 1
sll t2, t2, a4
slli t2, t2, 9
addi t2, t2, -1            // t2 = ((1 << a4) << 9) - 1
and a3, a3, t2
csetboundsexact ca0, ca0, a3
1:
```

## CSetBoundsRoundDown

CSetBoundsRoundDown can be tested in exactly the same way as CSetBounds but will have slightly different behaviour (rounding the bounds down instead of up).

### Pseudocode

```
new_address = cap.base + (rand_base % cap.range)
ca0.addr = new_address
max_increment = cap.top - cap.addr      // Calculate the new maximum length of the capability
if max_increment > 0:
    length = rand_length % max_increment
    new_cap = set_bounds(cap, top)
```

### Assembly

```asm
// Constrained CSetBoundsRoundDown routine that will maintain the tag bit of a given capability 
// while setting setting the bounds using two random words
//
// ca0    = capability to modify
// a2     = random 32-bit word
// a3     = random 32-bit word
// return = modified capability
valid_csetboundsrounddown:
cgetbase a4, ca0        
cgettop t0, ca0         
sub a4, t0, a4          // a3 = ca0.top - ca0.base
beqz a4, end            // if ca0.top - ca0.base == 0 then cannot shrink bounds so skip rest of test
remu a2, a2, a4         
cgetbase a4, ca0        
add a2, a2, a4          // a2 = ca0.base() + ( rand % ca0.length )
csetaddr ca0, ca0, a2
cgettop a4, ca0 
cgetaddr t0, ca0 
sub a4, a4, t0          // a4 = ca0.top - ca0.addr
beqz a4 end             // if ca0.top - ca0.addr == 0 then cannot shrink bounds so skip rest of test
remu a2 a3, a4          // length = rand_length % (ca0.top - ca0.addr)
csetboundsrounddown ca0, ca0, a2
end:
```

## CSetBoundsImm

CSetBoundsImm takes a capability `cap` and an unsigned immediate `IMM`.
The base of the result is set to `cap.addr` and the length to `IMM`.
This test checks whether the new bounds are within the bounds of `cap` and if so executes the instruction.

### Pseudocode

```
if cap.base < cap.addr + IMM < cap.top:
    CSetBoundsImm(cap, IMM)
```

### Assembly

```asm
// Constrained CSetBounds routine that will maintain the tag bit of a given capability 
// while setting setting the bounds using an immediate IMM
//
// ca0    = capability to modify
// return = modified capability
valid_csetboundsimm:
cgetaddr t0, ca0         
addi a4, t0, IMM         
cgettop t1, ca0          
bgeu a4, t1, 1f          // if cap.addr + imm > top: goto end
cgetbase t1, ca0         
bgtu t1, t0, 1f          // if cap.base > cap.addr: goto end
csetboundsimm ca0, ca0, IMM
1:
```

## CSetHigh

While `CSetHigh` always clears the tag bit, it can be used in conjunction with `CBuildCap` to build new tagged capabilities.
As this instruction always clears the tag bit, the test just runs the instruction (knowing that the tag bit will be cleared).

### Pseudocode

```
CSetHigh(cap, rand)
```

### Assembly

```asm
// CSetHigh routine that sets the high bits of a given capability
// using a random 32-bit word
//
// ca0    = capability to modify
// a2     = random 32-bit word
// return = modified capability
valid_csethigh:
csethigh ca0, ca0, a2
```

## CUnseal

CUnseal takes two capabilities, one that is sealed `cap` and one that is used to unseal it `unseal_cap`.
This test checks that the otype of `cap` is within the bounds of `unseal_cap` and if this is the case, `unseal_cap` is used to unseal `cap`.
There is no element of randomness in this test, as sealing is a binary operation that is or is not possible.

### Pseudocode

```
if unseal_cap.base <= cap.type < unseal_cap.top:
CUnseal(cap, cap, unseal_cap)
```

## Assembly

```asm
// Constrained CUnseal routine that will maintain the tag bit of a given capability 
// while unsealing it, if given another capability with permission to do so
//
// ca0    = sealed capability to unseal
// ca1    = capability with PERMIT_UNSEAL used to unseal capability in ca0
// return = modified capability
valid_cunseal:

// Check that the otype of cap is within the bounds on unseal_cap
cgettype a4,
ca0 cgetbase t0,
ca1
// if ca0.type < ca1.base end
bgeu t0,
a4,
1f

cgettop t0,
ca1 bgeu a4, t0,
1f

// The otype of cap is within the bounds on unseal_cap

cunseal ca0,
ca0,
ca1

// Moving values out of the registers
1:
```