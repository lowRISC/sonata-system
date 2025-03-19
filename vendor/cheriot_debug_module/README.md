
# RISC-V Debug Support for CHERIoT Ibex

This module is an implementation of a debug unit compliant with the [RISC-V
debug specification](https://github.com/riscv/riscv-debug-spec) v0.13.1. It is
a fork of the PULP Platform debug module which is used for the 
[Ariane](https://github.com/pulp-platform/ariane) and
[RI5CY](https://github.com/pulp-platform/riscv) cores.

To be compatible with the CHERIoT Ibex core the debug ROM and abstract command
implementation have been modified to replace the RV32I ISA instructions with
thier CHERIoT ISA equivilents. This work was done by SCI Semicondcutor.

## Implementation
We use an execution-based technique, also described in the specification, where
the core is running in a "park loop". Depending on the request made to the debug
unit via JTAG over the Debug Transport Module (DTM), the code that is being
executed is changed dynamically. This approach simplifies the implementation
side of the core, but means that the core is in fact always busy looping while
debugging.

## Features
The following features are currently supported

* Parametrizable buswidth for `XLEN=32` `XLEN=64` cores
  Note: CHERIoT is XLEN=32 only.
* Accessing registers over abstract command
* Program buffer
* System bus access (only `XLEN`)
* DTM with JTAG interface

These are not implemented (yet)

* Trigger module
* Quick access using abstract commands
* Accessing memory using abstract commands
* Authentication

## Tests

We use OpenOCD's [RISC-V compliance
tests](https://github.com/riscv/riscv-openocd/blob/riscv/src/target/riscv/riscv-013.c),
our custom testbench in `tb/` and
[riscv-tests/debug](https://github.com/riscv/riscv-tests/tree/master/debug).
