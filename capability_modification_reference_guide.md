# Reference Guide for Randomised Testing using Capability Modification Instructions
## 📚 Table of Contents

- [AUICGP](#auicgp)
- [AUIPCC](#auipcc)
- [CAndPerm](#candperm)
- [CClearTag](#ccleartag)
- [CGetAddr](#cgetaddr)
- [CGetBase](#cgetbase)
- [CGetHigh](#cgethigh)
- [CGetLen](#cgetlen)
- [CGetPerm](#cgetperm)
- [CGetTag](#cgettag)
- [CGetTop](#cgettop)
- [CGetType](#cgettype)
- [CIncAddr](#cincaddr)
- [CIncAddrImm](#cincaddrimm)
- [CJAL](#cjal)
- [CJALR](#cjalr)
- [CLC](#clc)
- [CMove](#cmove)
- [CRepresentableAlignmentMask](#crepresentablealignmentmask)
- [CRoundRepresentableLength](#croundrepresentablelength)
- [CSC](#csc)
- [CSeal](#cseal)
- [CSetAddr](#csetaddr)
- [CSetBounds](#csetbounds)
- [CSetBoundsExact](#csetboundsexact)
- [CSetBoundsRoundDown](#csetboundsrounddown)
- [CSetBoundsImm](#csetboundsimm)
- [CSetEqualExact](#csetequalexact)
- [CSetHigh](#csethigh)
- [CSpecialRW](#cspecialrw)
- [CSub](#csub)
- [CTestSubset](#ctestsubset)
- [CUnseal](#cunseal)
## AUICGP
## AUIPCC
## CAndPerm
## CClearTag
## CGetAddr
## CGetBase
## CGetHigh
## CGetLen
## CGetPerm
## CGetTag
## CGetTop
## CGetType
## CIncAddr
CIncOffset is an alias for this instruction.

Requires
- Random 32-bit word in a0
- Arbitrary capability in ca0
### Pseudocode
```
increment = rand_val % (cap.bounds + cap.base - cap.addr)
cap.addr += increment 
```
### Assembly
```asm
cgetlen a1, ca0 # a1 = ca0.bounds()
cgetaddr a2, ca0 # a2 = ca0.addr()
cgetbase a3, ca0 # a3 = ca0.base()
sub a2, a2, a3 # a2 = a2 - a3
sub a1, a1, a2 # a1 = a1 - a2 
remu a0 a0, a1 # a0 = a0 % a1
cincoffset ca0, ca0, a0 # ca0.addr += a0
```
This test guarantees that the address of the new capability is within the bounds of the old capability.
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
## CJAL
## CJALR
## CLC
## CMove
## CRepresentableAlignmentMask
## CRoundRepresentableLength
## CSC
## CSeal
## CSetAddr
## CSetBounds
Requires
- Random 32-bit word in a0
- Random 32-bit word in a1
- Arbitrary capability in ca0
### Pseudocode
```python
increment = rand_base % (cap.base + cap.bounds - cap.addr)
cap.addr += increment
length = rand_length % (cap.base + cap.bounds - cap.addr)
new_cap = set_bounds(cap, top)
```
### Assembly
```asm
cgetlen a2, ca0 # a2 = ca0.bounds
cgetaddr a3, ca0 # a3 = ca0.addr
cgetbase a4, ca0 # a4 = ca0.base
sub a4, a3, a4 # a2 = ca0.addr - ca0.base
sub a2, a2, a4 # a2 = a2 - a4 
remu a0 a0, a2 # a0 = a0 % a2
cincoffset ca0, ca0, a0 # ca0.addr += a0

remu a0 a1, a2 # a0 = a1 % a2
csetbounds ca0, ca0, a0
```
## CSetBoundsExact
## CSetBoundsRoundDown
## CSetBoundsImm
## CSetEqualExact
## CSetHigh
## CSpecialRW
## CSub
## CTestSubset
## CUnseal
