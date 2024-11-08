# Sonata release procedures

This document describes the process of creating a Sonata release.
It is mostly meant for lowRISC developers who are making a release, but it is useful to make this public for transparency reasons and possibly if others are having trouble with a release.
You'll need to use the rest of the documentation to set up your environment correctly, such as the CHERIoT toolchain and Python virtual environment using the [toolchain setup](toolchain-setup.md).
You'll also need a copy of Vivado.
Vivado version v2021.1 must be used for release creation and sign-off.
This matches the version used in CI and we have observed better QoR with this version vs more recent ones.
It can be obtained from the from the [Vivado Archive](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html).

## Hardware

Get the appropriate released version of the [rp2040 firmware](https://github.com/newaetech/sonata-rp2040/releases), and load it on to the board.
To load it on the board: unplug Sonata, hold SW9 while replugging, then drag the UF2 file you downloaded onto the `RPI-RP2` drive.

Setup release working area and check out fresh repository:

```shell
mkdir sonata-release
cd sonata-release
git clone https://github.com/lowRISC/sonata-system
cd sonata-system
git checkout $SONATA_SYSTEM_RELEASE_SHA
```

Build the bare-metal software including the bootloader:

```shell
export CHERIOT_LLVM_BIN=/location/of/llvm-cheriot/bin
cmake -B sw/cheri/build -S sw/cheri
cmake --build sw/cheri/build
```

Run the synthesis:

```shell
fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
```

Open Vivado GUI:

```shell
make -C ./build/lowrisc_sonata_system_0/synth-vivado build-gui
```

Check the following:
 - No timing failures (positive or 0 wns)
 - Synthesis and implementation errors (see known errors below)
 - Critical warnings (see known critical warnings below)

Create UF2 (alter output filename to have correct version number in place of X.Y):

```shell
uf2conv -b 0x00000000 -f 0x6ce29e6b ./build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit -co sonata-vX.Y.bit.slot1.uf2
uf2conv -b 0x10000000 -f 0x6ce29e6b ./build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit -co sonata-vX.Y.bit.slot2.uf2
uf2conv -b 0x20000000 -f 0x6ce29e6b ./build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit -co sonata-vX.Y.bit.slot3.uf2
# Copy bitstream UF2 to parent release dir
cp sonata-vX.Y.bit.slot1.uf2 ../
cp sonata-vX.Y.bit.slot2.uf2 ../
cp sonata-vX.Y.bit.slot3.uf2 ../
```

### Automated testing

Run tests on simulation (note python3.11 or above required). FPGA testing is currently done in CI and requires quite a few add-ons to your board.

```shell
# Build the simulator
fusesoc --cores-root=. run --target=sim --tool=verilator --setup --build lowrisc:sonata:system
# Run the tests
util/test_runner.py sim -e sw/cheri/build/tests/test_runner --simulator-binary build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator
```

Run tests on FPGA (note python3.11 or above required), adjust the /dev part to the UART as required. You will need to connect a Raspberry Pi sense HAT and a temperature sensor to QWIIC 1.

```shell
./util/test_runner.py fpga -e ./sw/cheri/build/tests/test_runner -t ./util/sonata-openocd-cfg.tcl /dev/ttyUSB2
```

### Bare-metal testing

CHERI Sanity:
```shell
./util/mem_helper.sh load_program -e sw/cheri/build/checks/cheri_sanity
```

You should see the user LEDs flashing, then press the joystick to see them flicker.

RGB LED test:
```shell
./util/mem_helper.sh load_program -e sw/cheri/build/checks/rgbled_test
```

Check the RGB LEDs turn on and change color in this sequence: green+red, blue+green, red+blue, off+off.

Run SPI flash test:
```shell
./util/mem_helper.sh load_program -e sw/cheri/build/checks/spi_test 
```

Open UART output:
```shell
screen /dev/ttyUSB2 921600
# Alternatively you can use picocom
picocom /dev/ttyUSB2 -b 921600 --imap lfcrlf
```

Output from UART:
```
JEDEC ID: ef 40 19 
Got first flash read:
00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 60 61 62 63 64 65 66 67 68 69 6a 6b 6c 6d 6e 6f 70 71 72 73 74 75 76 77 78 79 7a 7b 7c 7d 7e 7f 80 81 82 83 84 85 86 87 88 89 8a 8b 8c 8d 8e 8f 90 91 92 93 94 95 96 97 98 99 9a 9b 9c 9d 9e 9f a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 aa ab ac ad ae af b0 b1 b2 b3 b4 b5 b6 b7 b8 b9 ba bb bc bd be bf c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 ca cb cc cd ce cf d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 da db dc dd de df e0 e1 e2 e3 e4 e5 e6 e7 e8 e9 ea eb ec ed ee ef f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 fa fb fc fd fe ff 
Got second flash read:
80 81 82 83 84 85 86 87 88 89 8a 8b 8c 8d 8e 8f 90 91 92 93 94 95 96 97 98 99 9a 9b 9c 9d 9e 9f a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 aa ab ac ad ae af b0 b1 b2 b3 b4 b5 b6 b7 b8 b9 ba bb bc bd be bf c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 ca cb cc cd ce cf d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 da db dc dd de df e0 e1 e2 e3 e4 e5 e6 e7 e8 e9 ea eb ec ed ee ef f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 fa fb fc fd fe ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff 
```

Run the USB connection test:
```shell
util/mem_helper.sh load_program -e sw/cheri/build/checks/usbdev_check
```
Connect the user USB to a laptop's USB-A port. The UART should show this:
```
Initialising USB
Connected
Test passed; disconnected from USB.
```

Run the USB echo test. First you need to apply the following diff:
```diff
diff --git a/sw/cheri/checks/usbdev_check.cc b/sw/cheri/checks/usbdev_check.cc
index 4212ae7..7c68507 100644
--- a/sw/cheri/checks/usbdev_check.cc
+++ b/sw/cheri/checks/usbdev_check.cc
@@ -26,7 +26,7 @@ using namespace CHERI;
 // - if disconnection is not enabled then a terminator emulator may be attached to `/dev/ttyUSB<n>`
 //   and a sign-on message should be observed. Any characters entered into the terminal will
 //   be echoed on the UART output.
-static constexpr bool do_disconnect = true;
+static constexpr bool do_disconnect = false;
 
 static void write_strn(UartPtr uart, const char *str, size_t len) {
   while (len-- > 0u) {
```
Then use the following commands:
```shell
cmake --build sw/cheri/build
util/mem_helper.sh load_program -e sw/cheri/build/checks/usbdev_check
```

Check the UART output is:
```
Initialising USB
Connected
Sent sign-on message over USB.
```

Open USB serial output. For me this is `screen /dev/ttyUSB1`. Check that the output is:
```
Hello from CHERI USB!
```
Also type into the USB screen instance and see that it is echoed on the UART side.

#### Vivado - Known errors

None at the moment.

#### Vivado - Known critical warnings

None at the moment.

## Software repository

Check out fresh software repository:

```shell
cd ../
git clone https://github.com/lowRISC/sonata-software
cd sonata-software
git checkout $SONATA_SOFTWARE_RELEASE_SHA
git submodule update --init --recursive
```

Run through the instructions on the [Sonata software getting started page](https://lowrisc.github.io/sonata-software/doc/getting-started.html).
This must work with the latest release.
In particular we need these commands to work:

```shell
nix develop .
xmake -P ./examples
```

### Simple demo
Load simple demo on to board and make a copy for release:

```shell
# Copy for the release
cp ./build/cheriot/cheriot/release/sonata_simple_demo.slot1.uf2  ../sonata_simple_demo_vX.Y.slot1.uf2
# Program onto the FPGA
cp ../sonata_simple_demo_vX.Y.slot1.uf2 /path/to/SONATA
```

The LCD should display a lowRISC logo, 'Running on Sonata!' at the top and 'protected by CHERI' at the bottom.
The user LEDs should display a walking pattern.

Open screen:

```shell
screen /dev/ttyUSB2 921600
```

You should see output from the simple demo after you press the reset button (SW5):

```
bootloader: Loading software from flash...
bootloader: Booting into program, hopefully.
Led Walk Raw: Look pretty LEDs!
```

### Proximity test

Load proximity sensor demo on to board

```shell
cp ./build/cheriot/cheriot/release/sonata_proximity_demo.slot1.uf2 /path/to/SONATA
```

Ensure you have an APDS-9960 prox/gesture/color sensor plugged into qwiic1.

Check this has the same visual (LEDs and LCD) behaviour as the simple demo.
Wave your hand over the proximity sensor, you should see the RGB LEDs fade up and down (one on the left is red, one on the right is green) as you move your hand.
Red should fade up as your hand gets closer, green should fade down.
Check that the RGB LED colours are as expected.
This helps catch any issues that mix up the R,G,B values.

Open picocom:

```shell
screen /dev/ttyUSB2 921600
```

You should see output from the proximity demo:

```
proximity sensor example: Proximity is 0x12
proximity sensor example: Proximity is 0x11
proximity sensor example: Proximity is 0x16
proximity sensor example: Proximity is 0x19
proximity sensor example: Proximity is 0x1a
proximity sensor example: Proximity is 0x1c
proximity sensor example: Proximity is 0x1c
proximity sensor example: Proximity is 0x1a
proximity sensor example: Proximity is 0x18
```

These values will change as you move your hand over the proximity sensor.

### Snake demo

Load snake demo and play snake:

```shell
# Copy to parent release dir
cp ./build/cheriot/cheriot/release/snake_demo.slot1.uf2  ../snake_demo_vX.Y.slot1.uf2
# Program onto the FPGA
cp ../snake_demo_vX.Y.slot1.uf2 /path/to/SONATA/
```

Check that you see capability the exception LEDs light up and fade out when you hit the game boundaries ('tag' exception for top and left boundaries, 'bounds' exception for bottom and right boundaries).

### RTOS test suite

Run and build the CHERIoT RTOS test suite:

```shell
rm -r build .xmake
xmake config -P ./cheriot-rtos/tests/ --board=sonata-prerelease
xmake -P ./cheriot-rtos/tests/
llvm-strip ./build/cheriot/cheriot/release/test-suite -o test-suite.strip
uf2conv -b 0x00000000 -f 0x6ce29e60 test-suite.strip -co test-suite.slot1.uf2
cp test-suite.slot1.uf2 /path/to/SONATA
``` 

Results are output on the terminal (see instructions above), you will want to open that before you run the test suite otherwise you may miss it!

A successful run will output a large amount of text, it ends with something like:

```
Allocator test: fuzz i=0x0
Allocator test: fuzz i=0x8
Allocator test: fuzz i=0x10
Allocator test: fuzz i=0x18
Allocator test: fuzz i=0x20
Allocator test: fuzz i=0x28
Allocator test: fuzz i=0x30
Allocator test: fuzz i=0x38
Test runner: Allocator finished in 30477954 cycles
Test runner: All tests finished in 42895559 cycles
```

## Make Release

In your release directory from the procedures above you should have:
 - `sonata-vX.Y.bit.slot1.uf2`
 - `sonata_simple_demo_vX.Y.slot1.uf2`
 - `snake_demo_vX.Y.slot1.uf2`

Use these to create the GitHub release, remember to include the RP2040 UF2 and include appropriate release notes.

### Versioning

Choose a release number in the form "vX.Y" which is higher than the previous release.
Create a tag in sonata-system:
```shell
# Create git tag
git tag vX.Y
# Push this tag to upstream
git push --set-upstream upstream vX.Y
```

For the sonata-software repository we should create a branch:
```shell
git checkout -b vX.Y
git push --setupstream upstream vX.Y
```

### Release notes

Look for the previous tagged release and go through the commit history since then.
Note down any major updates like the additions of an IP block and make a bulleted list.
In Vivado look for the utilization report of the placed design, the timing summary of the routed design and the power report of the routed design to fill in the bitstream characteristics.
An example release notes looks something like this:

> This release contains on top of PREVIOUS_RELEASE:
>
> - MAJOR_FEATURE_ADDED_1
> - MAJOR_FEATURE_ADDED_2
> - ...
>
> Here are a few characteristics of this bitstream:
> - Utilization
>   * Slice LUTs XX.XX%
>   * Slice Registers: XX.XX%
>   * Block RAM Tile: XX.XX%
>   * DSPs: XX.XX%
> - Timing
>   * Overall WNS: X.XXX ns
>   * System clock WNS: X.XXX ns
>   * HyperRAM clock WNS: X.XXX ns
>   * USB clock WNS: X.XXX ns
> - Total on-chip power: X.XXX W
>
> Here's the developer flow for using these files:
>
> 1. Make sure the bitstream select switch (immediately below the main USB-C port) is set to position 1.
> 2. Before plugging in your Sonata board, hold down the "SW9" button labelled "RP2040 Boot", and while holding this button plug your board into your laptop using the Main USB.
> 3. A drive called "RPI-RP2" should pop up on your computer and copy `rpi_rp2_vX.Y.uf2` into it.
> 4. This drive should automatically dismount once the file is transferred and remount as "SONATA". Once the remount has happened, you can drag in the wrapped bitstream `sonata_bitstream_vX.Y.bit.slot1.uf2`.
> 5. Once programming is successful, you should see the "CHERI" LED light up and the "LEGACY" LED turn off. If this is not the case, the bitstream loading may have failed and you should retry by unplugging and replugging the main USB on the Sonata board. Then re-drag the bitstream into the "SONATA" drive.
> 6. After programming the bitstream, drag the `sonata_simple_demo_vX.Y.slot1.uf2` into the "SONATA" drive.
> 7. You should now see the user LEDs turn on and off, as well as the lowRISC logo appear on the LCD.
> 8. You can also drag `snake_demo_vX.Y.slot1.uf2` into the SONATA drive to play snake using the joystick. Watch the CHERI error LEDs as you hit the boundary.
