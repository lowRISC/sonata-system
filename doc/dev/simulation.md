# Simulation

The Sonata simulation environment uses Verilator.

## Building

Use the following command to build the simulator binary.
The resulting executable will be available at `./result/bin/sonata-simulator`.

```sh
nix build .#sonata-simulator
```


If you'd like more control over the build, you can manually invoke FuseSoC with the following.
The resulting executable can be found at `./build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator`.

```sh
NUM_CORES=4
fusesoc --cores-root=. run \
  --target=sim --tool=verilator --setup \
  --build lowrisc:sonata:system \
  --verilator_options="-j $NUM_CORES" \
  --make_options="-j $NUM_CORES"
```

*To enable tracing append, `--verilator_options='+define+RVFI'` to the command above.*

## Notes on building natively

**This is only for advanced users.**

If for some reason you do not want to use the nix environment and need to set up your own environment, here are some commands that you can use if you're running using podman or Docker:

```Dockerfile
# Download LLVM toolchain
FROM ubuntu:24.04 as llvm-download
RUN apt update && apt install -y curl unzip
RUN curl -O https://api.cirrus-ci.com/v1/artifact/github/CHERIoT-Platform/llvm-project/Build%20and%20upload%20artefact%20$(uname -p)/binaries.zip
RUN unzip binaries.zip

# Build Verilator
FROM ubuntu:24.04 AS verilator-build
# Verilator dependencies
RUN apt update && apt install -y git help2man perl python3 make g++ libfl2 libfl-dev zlib1g zlib1g-dev autoconf flex bison
WORKDIR /
# Build Verilator
RUN git clone https://github.com/verilator/verilator
WORKDIR verilator
RUN git checkout v5.024
RUN mkdir install
RUN autoconf \
    && ./configure --prefix=/verilator/install \
    && make -j `nproc` \
    && make install

# Build Sonata simulator
FROM ubuntu:24.04 as sonata-build
# Sonata dependencies
RUN apt update && apt install -y git python3 python3-venv build-essential libelf-dev libxml2-dev
# Install LLVM for sim boot stub.
RUN mkdir -p /cheriot-tools/bin
COPY --from=llvm-download "/Build/install/bin/clang-13" "/Build/install/bin/lld" "/Build/install/bin/llvm-objcopy" "/Build/install/bin/llvm-objdump" "/Build/install/bin/clangd" "/Build/install/bin/clang-format" "/Build/install/bin/clang-tidy" /cheriot-tools/bin/
# Create the LLVM tool symlinks.
RUN cd /cheriot-tools/bin \
    && ln -s clang-13 clang \
    && ln -s clang clang++ \
    && ln -s lld ld.lld \
    && ln -s llvm-objcopy objcopy \
    && ln -s llvm-objdump objdump \
    && chmod +x *
COPY --from=verilator-build "/verilator/install" /verilator
WORKDIR /
# Build Sonata simulator
RUN git clone https://github.com/lowRISC/sonata-system
WORKDIR sonata-system
RUN python3 -m venv .venv \
    && . .venv/bin/activate \
    && pip install -r python-requirements.txt \
    && export PATH=/verilator/bin:$PATH \
    && fusesoc --cores-root=. run --target=sim --tool=verilator --setup --build lowrisc:sonata:system
RUN cp build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator /sonata_simulator
# Build Sonata simulator boot stub
WORKDIR sw/cheri/sim_boot_stub
RUN export PATH=/cheriot-tools/bin:$PATH \
    && make
RUN cp sim_boot_stub /sonata_simulator_boot_stub
```

## Running baremetal

Running the simulator can be accomplished with the following command, where you can change the `meminit` argument to a different program if you wish:
```sh
./build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator -t --meminit=ram,./sw/cheri/cheri_sanity/boot.elf
```

I recommend that you make the following change to the sanity check to see quicker changes in simulation:
```diff
diff --git a/sw/cheri/cheri_sanity/boot.cc b/sw/cheri/cheri_sanity/boot.cc
index 547abb3..7f7781d 100644
--- a/sw/cheri/cheri_sanity/boot.cc
+++ b/sw/cheri/cheri_sanity/boot.cc
@@ -32,7 +32,7 @@ extern "C" uint32_t rom_loader_entry(void *rwRoot)
        uint32_t switchValue = 0;
        while (true) {
                gpioValue ^= GPIO_VALUE;
-               for (int i = 0; i < 5000000; i++) {
+               for (int i = 0; i < 5; i++) {
                        switchValue = *((volatile uint32_t *) gpi);
                        switchValue <<= 4; // shift input onto LEDs and skipping LCD pins
                        *((volatile uint32_t *) gpo) = gpioValue ^ switchValue;
```

## Running with boot stub

The [Sonata software repository](https://github.com/lowRISC/sonata-software) assumes you have a boot loader that jumps to `0x00101000`.
To help with this we have a `sim_boot_stub`, which you can build as follows assuming you have access to CHERIoT LLVM:
```sh
make -C sw/cheri/sim_boot_stub
```

You can then run your example code through the simulator as follows:
```sh
./build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator -t -E sw/cheri/sim_boot_stub/sim_boot_stub -E /path/to/sonata-software/build/cheriot/cheriot/release/sonata_simple_demo
```

## Debugging

If you want to look at the internal design in more details, you can explore the waveforms produced by the simulation using [GTKWave](http://gtkwave.sourceforge.net/):
```sh
gtkwave sim.fst data/pc_and_gpo.gtkw
```
