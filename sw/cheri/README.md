# CHERIoT programs for the Sonata system

***The software in the `sonata-system` repository is chiefly for hardware development and testing.***
It is not recommended that sonata users use this software to explore the system.
Users would be better served by the [`cheriot-rtos`][] and [`sonata-software`][] repositories.

[`cheriot-rtos`]: https://github.com/CHERIoT-Platform/cheriot-rtos
[`sonata-software`]: https://github.com/lowRISC/sonata-software

See the [getting started guide](../../doc/guide/README.md) for how to install dependencies.

## Directory Structure
- `boot` contains the sonata bootloader that loads a program from flash on start-up.
- `sim_boot_stub` contains a shim bootloader so can be given the sonata simulator so that it jumps to the expected program start address.
- `checks` contains pieces of software that have hardware side effects, e.g. turning an LED on.
    These can be used by hardware developers to check functionality.
- `tests` contains the test suite. This can be run in the simulator or on the FPGA to test hardware.
    Unlike the checks above, return whether they have passed or not over the primary UART.
- `common` contains that maybe useful for multiple different targets.

## Software Runtime
The software runtime is very minimal.
Not only is there no allocator, but it does not work around certain CHERIoT features.
For example, one cannot link multiple object files because the linker will each to be compartmentalised, and they aren't in this environment.


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
    --SRAMInitFile=$ABSOLUTE_PATH_TO/build_cheri/checks/cheri_sanity.vmem
```

## Loading software onto the FPGA

You can load software onto the FPGA over JTAG using:

```sh
./util/mem_helper.sh load_program -e sw/cheri/build/checks/spi_test
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
