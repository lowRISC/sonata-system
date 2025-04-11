# FPGA development

This page is only for if you want to make changes to the RTL of the bitstream.
In most cases you can just use the standard bitstream published in the [releases](https://github.com/lowRISC/sonata-system/releases).

## Dependencies

- [Vivado](https://www.xilinx.com/support/download.html)
- [OpenFPGALoader](https://github.com/trabucayre/openFPGALoader)

## FPGA Build

The Sonata bitstream is generated using Vivado.

## Bitstream

To build the bitstream, make sure to [build the baremetal software](../guide/building-examples.md#baremetal-examples) to create the correct SRAM image.
Then run this command:

```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
```

You can also manually set the initial value of the SRAM, for example:

```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system --SRAMInitFile=$PWD/sw/cheri/build/tests/uart_check.vmem
```
## Build bitstream using nix
Optionally, the bitstream can be built using a nix command:
```sh
nix run .#bitstream-build
```
Note: Vivado must be in one's path for this command to work.

## Sonata XL bitstream

To build a bitstream for the Sonata XL board instead of the Sonata One board, change the fusesoc target from `synth` to `synth_xl`.
i.e. run this command:

```sh
fusesoc --cores-root=. run --target=synth_xl --setup --build lowrisc:sonata:system
```

This bitstream should be compatible with software compiled for the Sonata One board.
A faux-HyperRAM is instantiated using the additional block RAM of the larger FPGA to account for the lack of HyperRAM onboard.
