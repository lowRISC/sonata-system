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
- Random capability in ca0
```asm
cgetlen a1, ca0 # a1 = ca0.bounds()
remu a0 a0, a1 # a0 = a0 % a1
cincoffset ca0, ca0, a0 # ca0.addr += a0
```
This test guarantees that the address of the new capability is within the bounds of the old capability.
## CIncAddrImm
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
## CSetBoundsExact
## CSetBoundsRoundDown
## CSetBoundsImm
## CSetEqualExact
## CSetHigh
## CSpecialRW
## CSub
## CTestSubset
## CUnseal
