# Getting started

This guide helps you to get started with Sonata development.

## Dependencies

- [Vivado](https://www.xilinx.com/support/download.html)
- [OpenFPGALoader](https://github.com/trabucayre/openFPGALoader)
- [rv32imc GCC toolchain](https://github.com/lowRISC/lowrisc-toolchains/releases)
  (For example: `lowrisc-toolchain-rv32imcb-20220524-1.tar.xz`)
- cmake
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

Building the software:
```sh
mkdir sw/c/build
pushd sw/c/build
cmake ..
make
popd
```

## Simulation

The Sonata simulation environment uses Verilator.

### Building

Use the following FuseSoC command to build the simulator binary:
```sh
fusesoc --cores-root=. run --target=sim --tool=verilator --setup --build lowrisc:sonata:system
```

### Running

Running the simulator can be accomplished with the following command, where you can change the `meminit` argument to a different program if you wish:
```sh
./build/lowrisc_sonata_system_0/sim-verilator/Vsonata_system -t --meminit=ram,./sw/c/build/test/memory_test
```

### Debugging

If you want to look at the internal design in more details, you can explore the waveforms produced by the simulation using [GTKWave](http://gtkwave.sourceforge.net/):
```sh
gtkwave sim.fst
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

### Programming

Programming the FPGA:
```sh
openFPGALoader -b arty_a7_35t build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit
```
