# Setting up the Toolchain for Software Development

All the special CHERIoT goodness comes with its own compiler that understands how to use it. For this reason you'll need to build a
special toolchain from source. Luckily, it should be easy if you follow our simple instructions.

If building on Windows, the following instructions have also been confirmed to work with WSL2 with the exception of `edalize` and `fusesoc`,
which *are not required for software development*.

## Sonata Setup

```sh
git clone https://github.com/lowRISC/sonata-system
cd sonata-system

# Setup python venv
python3 -m venv .venv
source .venv/bin/activate

# Install python requirements
pip3 install -r python-requirements.txt
```

> This installs requirements for both software and FPGA development. You may get errors on `edalize` and `fusesoc` --
> don't panic, you don't need those for software. Just ignore the errors, you should see the rest of the packages
> installed successfully.

In the future, if you dare close this terminal, you'll need to do this before building Sonata examples:

```sh
cd sonata-system
source .venv/bin/activate
```

## Building Toolchain

To build the toolchain, you'll need:

* clang
* ninja-build
* lld (llvm linker)
* cmake

On Ubuntu (including WSL), you can install them with:

``sudo apt get install clang ninja-build lld cmake``

> HINT: You can see all the commands used to setup the test running in the
> [CI YAML file](https://github.com/CHERIoT-Platform/llvm-project/blob/cheriot/.cirrus.yml). This provides a set of commands that
> is tested on each commit, in case you are having trouble building anything and want to see the expected output.

Build the toolchain with (again be sure this is in the `.venv` terminal):

```sh
git clone --depth 1 https://github.com/CHERIoT-Platform/llvm-project cheriot-llvm
cd cheriot-llvm
git checkout cheriot
# Create build directory
export LLVM_PATH=$(pwd)
mkdir -p builds/cheriot-llvm
cd builds/cheriot-llvm
# Build the toolchain
cmake ${LLVM_PATH}/llvm -DCMAKE_BUILD_TYPE=Release -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" -DCMAKE_INSTALL_PREFIX=install -DLLVM_ENABLE_UNWIND_TABLES=NO -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_DISTRIBUTION_COMPONENTS="clang;clangd;lld;llvm-objdump;llvm-objcopy" -G Ninja
export NINJA_STATUS='%p [%f:%s/%t] %o/s, %es '
ninja install-distribution
```

> NOTE: Currently the WSL2 build is broken and requires a patch applied as follows:
> ```sh
> wget https://github.com/llvm/llvm-project/commit/4ad9ec8a328ccb3b836c993bba954366f05b2fd4.patch
> git am < 4ad9ec8a328ccb3b836c993bba954366f05b2fd4.patch
> ```

Note the checkout and build will take some time, and the build process may have limited output during some steps.

This should put the binaries in a `bin` subdirectory of your build folder (the `builds/cheriot-llvm` folder you made).

With the CHERIoT/LLVM toolchain built, you can now continue to setup the examples and build them.
