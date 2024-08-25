# Building Examples

The following contains some simple examples you can build for the Sonata board.
Once you've got these builds working, you can easily add more features to the example code.

## Using our template

Please go to the [Sonata software repository](https://github.com/lowRISC/sonata-software) using the branch appropriate to your release (e.g. checkout the ['v0.3' branch](https://github.com/lowRISC/sonata-software/tree/v0.3)) and build a full application from there.
Inside your setup you should simply be able to build it like this:

```sh
git clone --recurse-submodule -b v0.3 \
    https://github.com/lowRISC/sonata-software.git
cd sonata-software
nix develop .
xmake -P examples/
```

After running this you should see the build run to completion and report success, the critical lines indicating a successful build are (note output size may differ):

```sh
Converted to uf2, output size: 74752, start address: 0x101000
Wrote 74752 bytes to build/cheriot/cheriot/release/sonata_simple_demo.uf2
[100%]: build ok, spent 6.827s
```

You can drag and drop this UF2 file into the `SONATA` drive to program the firmware.

## UART output

On Linux use the following command to check you can receive serial output:
```sh
screen /dev/ttyUSB2 115200
```

On Mac this is similar
```sh
screen /dev/tty.usbserial-LN100302 115200
```

On Windows, connecting to serial ports directly from within WSL2 (default) is not possible.
Connecting from WSL1 is possible, but we recommend to use [PuTTY](https://www.putty.org/) to connect to serial ports.
Alternatively you can use [Termite](https://www.compuphase.com/software_termite.htm).

Select "Serial" as "Connection type", put the COM port in the "Serial line" text field, and set "Speed" to 921600.
To find out what serial ports are available, you can open Device Manager and all connected serial ports are listed under "Ports (COM & LPT)" section.

Note that the baremetal examples below use a higher baud rate, 921600, use that in place of 115200 when running the baremetal examples.

## Baremetal examples

This is **only for advanced users**.
If you want to build the baremetal examples in the Sonata repo you can follow these instructions.

First [setup a toolchain](../dev/toolchain-setup.md).
Note none of the current Nix environments have exactly the correct set of dependencies to build the baremetal examples.
This will change but for the time being you either need to alter a Nix environment (either add cmake to the sonata-software environment or the CHERIoT toolchain to the sonata-system environment) or setup a toolchain outside Nix.


### Additional Toolchain Setup

Besides the compiler, there are a few more features the example code depends on.

#### SRecord Tools

The build environment uses srecord tools, which you can install with:

```bash
sudo apt install srecord
```

#### CHERIoT LLVM

**IMPORTANT**: Set this environmental variable to the llvm build you did in the previous part:

```sh
export CHERIOT_LLVM_BIN=/path/to/cheriot-llvm/bin
```

> WARNING: The path to `/path/to/cheriot-llvm/bin` should point to the *build* directory you created, not just the root checked out `cheriot-llvm` directory.
> The path will look something like: `~/llvm-tools/cheriot-llvm/builds/cheriot-llvm/bin`

Please make sure you unset the `CHERIOT_RTOS_SDK` environment variable as `cmake` will automatically pull in [the correct version](https://github.com/lowRISC/cheriot-rtos/tree/sonata).

The following assumes you have run the `source .venv/bin/activate` command in the terminal you are using, and you are currently at the root directory of your local `sonata-system` git repository.

### Building Baremetal Examples

To build, run the following from the root of the directory which will build the examples:

```
cmake -B sw/cheri/build -S sw/cheri 
cmake --build sw/cheri/build
```
The build output is put in the `sw/cheri/build` directory.
Two files of interest are created for each target: an ELF file which has no extension and a `*.vmem` file.
The `*.vmem` file can be used to load directly into the FPGA bitstream, described in more detail on the [Programming the Sonata Software](../dev/sw-programming.md) page.

> If you get an error that `CMake will not be able to correctly generate this project.`, check back in the list to see if you see an error within the output similar to `clang: error: unknown argument: '-mxcheri-rvc'`.
> If this happens it means the wrong (non-CHERIoT) compiler was used.
> Check back to see what compiler is being used.

### Loading software onto the FPGA

You can load software onto the FPGA over USB (JTAG) using:

```sh
./util/mem_helper.sh load_program -e sw/cheri/build/tests/spi_test
```
There are actually four different ways of loading the program - we normally use JTAG for development, but you can also program it into the serial flash device on the board.
See the page [Programming the Sonata Software](../dev/sw-programming.md).
