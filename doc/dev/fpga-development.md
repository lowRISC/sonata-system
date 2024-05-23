# FPGA development

This page is only for if you want to make changes to the RTL of the bitstream.
In most cases you can just use the standard bitstream published in the [releases](https://github.com/lowRISC/sonata-system/releases).

## Dependencies

- [Vivado](https://www.xilinx.com/support/download.html)
- [OpenFPGALoader](https://github.com/trabucayre/openFPGALoader)

## FPGA Build

The Sonata bitstream is generated using Vivado.

## Bitstream

To build the bitstream, make sure to [build the software](../guide/building-examples.md) to create the correct SRAM image.
Then run this command:

```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
```

You can also manually set the initial value of the SRAM, for example:

```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system --SRAMInitFile=$PWD/sw/cheri/build/tests/uart_check.vmem
```

