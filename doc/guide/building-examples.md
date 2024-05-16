# Building Examples

The following contains some simple examples you can build for the Sonata board. Once you've got these builds working, you can easily add more features to the example code.

## Additional Toolchain Setup

Besides the compiler, there are a few more features the example code depends on:

### SRecord Tools

The makefile assumes srecord tools, which you can install with:

```bash
sudo apt install srecord
```

### CHERIoT RTOS SDK Installation

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

## Building All Examples

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