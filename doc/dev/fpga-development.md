# FPGA development

## Dependencies

- [Vivado](https://www.xilinx.com/support/download.html)
- [OpenFPGALoader](https://github.com/trabucayre/openFPGALoader)

## FPGA Build

The Sonata bitstream is generated using Vivado.

## Bitstream

To build the bitstream, make sure to follow the [Software section](#software) to create the correct SRAM image.
Then run this command:
```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
```


### Programming the Bitstream

```sh
openFPGALoader -c ft4232 build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit
```
