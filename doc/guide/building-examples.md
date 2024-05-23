# Building Examples

The following contains some simple examples you can build for the Sonata board. Once you've got these builds working, you can easily add more features to the example code.

## Using our template

Please go to the [Sonata software repository](https://github.com/lowRISC/sonata-software) and build a full application from there.
Inside your setup you should simply be able to build it like this:

```sh
git clone --recurse-submodule \
    https://github.com/lowRISC/sonata-software.git
cd sonata-software
xmake config --board=sonata
xmake build
```

After running this you should see the build run to completion and report success, the critical lines indicating a successful build are (note output size may differ):

```sh
Converted to uf2, output size: 74752, start address: 0x101000
Wrote 74752 bytes to build/cheriot/cheriot/release/sonata_simple_demo.uf2
[100%]: build ok, spent 6.827s
```

You can drag and drop this UF2 file into the `SONATA` drive to program the firmware.

## Baremetal examples

This is only for advanced users.
If you want to build the baremetal examples in the Sonata repo you can follow these instructions.

### Additional Toolchain Setup

Besides the compiler, there are a few more features the example code depends on:

#### SRecord Tools

The makefile assumes srecord tools, which you can install with:

```bash
sudo apt install srecord
```

#### CHERIoT RTOS SDK Installation

You will need a copy of [CHERIoT RTOS](https://github.com/microsoft/cheriot-rtos/tree/main) for this section.

On Windows you may need to set `git config --global core.symlinks true` *before* cloning the repository.

Clone the repository somewhere, *not* into the root of the `sonata-system` directory, but at the same level as `sonata-system`:

```sh
cd ..
git clone --recurse https://github.com/microsoft/cheriot-rtos.git
```

**IMPORTANT**: Set these two environmental variables. The first should point to the llvm build you did in the previous part,
and the second should point to the `cheriot-rtos` repo you just checked out:

```sh
export CHERIOT_LLVM_BIN=/path/to/cheriot-llvm/bin
export CHERIOT_RTOS_SDK=/path/to/cheriot-rtos/sdk
```

> WARNING: The path to `/path/to/cheriot-llvm/bin` should point to the *build* directory you created, not just the root checked out `cheriot-llvm` directory.
> The path will look something like: `~/llvm-tools/cheriot-llvm/builds/cheriot-llvm/bin`

The following assume you have run the `source .venv/bin/activate` command in the terminal you are using, and you are
currently at the root directory of your local `sonata-system` git repository.

### Building Baremetal Examples

> TODO: We should tell them where to get the SW. And especially point out it must match
> the bitstream version again (all downloaded at one point).

> TODO: These environmental variables don't seem to be used by CMAKE but would need to be based.
> So these instructions are broken but kept as a starting point.

To build, run the following from the root of the directory which will build the examples:

```
cmake -B sw/cheri/build -S sw/cheri 
cmake --build sw/cheri/build
```
The build output is put in the `sw/cheri/build` directory.
Two files of interest are created for each target: an ELF file which has no extension and a `*.vmem` file. The
`*.vmem` file can be used to load directly into the FPGA bitstream, described in more detail on the [Programming the Sonata Software](../dev/sw-programming.md) page.

> If you get an error that `CMake will not be able to correctly generate this project.`, check
> back in the list to see if you see an error within the output similar to 
> `clang: error: unknown argument: '-mxcheri-rvc'`. If this happens it means the wrong (non-CHERIoT)
> compiler was used. Check back to see what compiler is being used.

### Loading software onto the FPGA

You can load software onto the FPGA over USB (JTAG) using:

```sh
./util/mem_helper.sh load_program -e sw/cheri/build/tests/spi_test
```
There are actually four different ways of loading the program - we normally use JTAG for development, but you can also
program it into the serial flash device on the board. See the page [Programming the Sonata Software](../dev/sw-programming.md).
