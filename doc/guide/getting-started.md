# Getting started

This guide helps you to get started with Sonata development.

## Dependencies

- [Vivado](https://www.xilinx.com/support/download.html)
- [OpenFPGALoader](https://github.com/trabucayre/openFPGALoader)
- [CHERIoT toolchain](https://github.com/CHERIoT-Platform/llvm-project/tree/cheriot)
- cmake
- ninja
- python3 - Additional python dependencies in python-requirements.txt installed with pip
- openocd (version 0.11.0 or above)
- screen
- srecord

### Python

Setting up your Python environment:
```sh
# Setup python venv
python3 -m venv .venv
source .venv/bin/activate

# Install python requirements
pip3 install -r python-requirements.txt
```

## Software

### Toolchain
If these instructions ever go out of date, you should be able to find the up to date instructions to build the toolchain [from the CI YAML file](https://github.com/CHERIoT-Platform/llvm-project/blob/cheriot/.cirrus.yml).

Here are the insructions I used to build the LLVM toolchain:
```sh
# Checkout the repository
git clone --depth 1 https://github.com/CHERIoT-Platform/llvm-project cheriot-llvm
cd cheriot-llvm
git checkout cheriot
# Create build directory
export LLVM_PATH=$(pwd)
mkdir -p builds/cheriot-llvm
cd builds/cheriot-llvm
# Build the toolchain
cmake ${LLVM_PATH}/llvm -DCMAKE_BUILD_TYPE=Release -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" -DCMAKE_INSTALL_PREFIX=install -DLLVM_ENABLE_UNWIND_TABLES=NO -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_DISTRIBUTION_COMPONENTS="clang;clangd;lld;llvm-objdump;llvm-objcopy" -G Ninja
export NINJA_STATUS='%p [%f:%s/%t] %o/s, %es'
ninja install-distribution
```

### Building capability sanity check
You will need a copy of [CHERIoT RTOS](https://github.com/microsoft/cheriot-rtos/tree/main) for this section.

Building the software:
```sh
pushd sw/cpp/cheri_sanity
make CHERIOT_LLVM_ROOT=/path/to/cheriot-llvm/bin CHERIOT_RTOS_SDK=/path/to/cheriot-rtos/sdk
popd
```

## Simulation

The Sonata simulation environment uses Verilator.

### Building

Use the following FuseSoC command to build the simulator binary:
```sh
fusesoc --cores-root=. run --target=sim --tool=verilator --setup --build lowrisc:sonata:system
```

*To enable tracing append, `--verilator_options='+define+RVFI'` to the command above.*

### Running

Running the simulator can be accomplished with the following command, where you can change the `meminit` argument to a different program if you wish:
```sh
./build/lowrisc_sonata_system_0/sim-verilator/Vsonata_system -t --meminit=ram,./sw/cpp/cheri_sanity/boot.elf
```

I recommend that you make the following change to the sanity check to see quicker changes in simulation:
```diff
diff --git a/sw/cpp/cheri_sanity/boot.cc b/sw/cpp/cheri_sanity/boot.cc
index 547abb3..7f7781d 100644
--- a/sw/cpp/cheri_sanity/boot.cc
+++ b/sw/cpp/cheri_sanity/boot.cc
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

### Debugging

If you want to look at the internal design in more details, you can explore the waveforms produced by the simulation using [GTKWave](http://gtkwave.sourceforge.net/):
```sh
gtkwave sim.fst data/pc_and_gpo.gtkw
```

## FPGA

The Sonata bitstream is generated using Vivado.

### USB rules for Linux

The following instructions will only work on Linux.

To interact with the Arty-A7, please add the following rules:
```sh

sudo su
cat <<EOF > /etc/udev/rules.d/90-arty-a7.rules
# Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC
# used on Digilent boards
ACTION=="add|change", SUBSYSTEM=="usb|tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ATTRS{manufacturer}=="Digilent", MODE="0666"

# Future Technology Devices International, Ltd FT232 Serial (UART) IC
ACTION=="add|change", SUBSYSTEM=="usb|tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"
EOF

exit
```

To allow openFPGAloader to program our device, add the following rules:
```sh
sudo su
cat <<EOF > /etc/udev/rules.d/99-openfpgaloader.rules
# Copy this file to /etc/udev/rules.d/

ACTION!="add|change", GOTO="openfpgaloader_rules_end"

# gpiochip subsystem
SUBSYSTEM=="gpio", MODE="0664", GROUP="plugdev", TAG+="uaccess"

SUBSYSTEM!="usb|tty|hidraw", GOTO="openfpgaloader_rules_end"

# Original FT232/FT245 VID:PID
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="664", GROUP="plugdev", TAG+="uaccess"

# Original FT2232 VID:PID
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="664", GROUP="plugdev", TAG+="uaccess"

# Original FT4232 VID:PID
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", MODE="664", GROUP="plugdev", TAG+="uaccess"

# Original FT232H VID:PID
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="664", GROUP="plugdev", TAG+="uaccess"

# Original FT231X VID:PID
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE="664", GROUP="plugdev", TAG+="uaccess"

# anlogic cable
ATTRS{idVendor}=="0547", ATTRS{idProduct}=="1002", MODE="664", GROUP="plugdev", TAG+="uaccess"

# altera usb-blaster
ATTRS{idVendor}=="09fb", ATTRS{idProduct}=="6001", MODE="664", GROUP="plugdev", TAG+="uaccess"
ATTRS{idVendor}=="09fb", ATTRS{idProduct}=="6002", MODE="664", GROUP="plugdev", TAG+="uaccess"
ATTRS{idVendor}=="09fb", ATTRS{idProduct}=="6003", MODE="664", GROUP="plugdev", TAG+="uaccess"

# altera usb-blasterII - uninitialized
ATTRS{idVendor}=="09fb", ATTRS{idProduct}=="6810", MODE="664", GROUP="plugdev", TAG+="uaccess"
# altera usb-blasterII - initialized
ATTRS{idVendor}=="09fb", ATTRS{idProduct}=="6010", MODE="664", GROUP="plugdev", TAG+="uaccess"

# dirtyJTAG
ATTRS{idVendor}=="1209", ATTRS{idProduct}=="c0ca", MODE="664", GROUP="plugdev", TAG+="uaccess"

# Jlink
ATTRS{idVendor}=="1366", ATTRS{idProduct}=="0105", MODE="664", GROUP="plugdev", TAG+="uaccess"

# NXP LPC-Link2
ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0090", MODE="664", GROUP="plugdev", TAG+="uaccess"

# NXP ARM mbed
ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", MODE="664", GROUP="plugdev", TAG+="uaccess"

# icebreaker bitsy
ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6146", MODE="664", GROUP="plugdev", TAG+="uaccess"

# orbtrace-mini dfu
ATTRS{idVendor}=="1209", ATTRS{idProduct}=="3442", MODE="664", GROUP="plugdev", TAG+="uaccess"

LABEL="openfpgaloader_rules_end"

EOF

exit

```

Run the following to reload the rules:
```sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Add user to plugdev group:
```sh
sudo usermod -a $USER -G plugdev
```

### Bitstream

To build the bitstream, make sure to follow the [Software section](#software) to create the correct SRAM image.
Then run this command:
```sh
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
```

For the Arty A7 use the `synth_artya7` target.

### Programming

Programming the FPGA:
```sh
openFPGALoader -c ft4232 build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit
```

For the Arty A7 use `-b arty_a7_35t` instead of `-c ft4232`.
