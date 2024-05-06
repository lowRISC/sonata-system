# CHERIoT programs for the Sonata system

See the [getting started guide](../../doc/guide/getting-started.md) for how to install dependancies.

## Building

To build, run the following from the root of the directory.

```
cmake -B sw/cheri/build -S sw/cheri
cmake --build sw/cheri/build
```

The build output is put in the `sw/cheri/build` directory.
Two files of interest are created for each target: an ELF file which has no extension and a `*.vmem` file.


## Embedding software into a bitstream

When building a bit stream, you can override the default software by changing the `SRAMInitFile` parameter to point to the `.vmem` file you'd prefer.

```sh
fusesoc --cores-root=. run --target=synth \
    --setup --build lowrisc:sonata:system \
    --SRAMInitFile=$ABSOLUTE_PATH_TO/build_cheri/tests/cheri_sanity.vmem
```

## Loading software onto the FPGA

You can load software onto the FPGA over JTAG using:

```sh
./util/mem_helper.sh load_program -e sw/cheri/build/tests/spi_test
```


## Programs
### Sanity check

The `cheri_sanity` program contains the code necessary to test pure capability mode on the Sonata system:
- Store data (by changing LEDs)
- Load data (by reading switches)
- Store capability to SRAM
- Load capability from SRAM


### CHERI error test for Sonata

The `error_leds` program triggers each type of CHERIoT exception.
See Table 7.3 in the [CHERIoT ISA](https://www.microsoft.com/en-us/research/uploads/prod/2023/02/cheriot-63e11a4f1e629.pdf).
