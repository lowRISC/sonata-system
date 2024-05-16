# Reloading the FPGA Image

The first thing you should do before building the firmware is to get the latest version of the FPGA image, called the "bitstream". This contains the configuration
for the microcontroller core & peripherals. The "release version" of the bitstream *must* match the
configuration you use to build the software, as if the bitstream is a different version than what
the software is expecting, you are not going to have fun!

When you download a release from the Sonata System page, you'll have a matching bitstream and
software setup.

While you can build your own bitstream as described
in [FPGA Development](../dev/fpga-development.md), we recommend starting with our prebuilt bitstream first. Building the bitstream requires
installing Vivado which takes a large amount of hard drive space and requires a seperate manual installation process (as well as the build
process is much slower than a software compile, so adds delay until you can play with CHERIoT).

## Selecting a Bitstream

When the Sonata board is plugged in, it loads one of three bitstreams. This is selected by the switch below the USB port labeled `Bitstream`:

![](img/sonata-selectbs.jpeg)

The LEDs besides the switch show the current image selected as well for confirmation. We recommend using Slot 2 (the middle setting), leaving Slot 1 as the test image we shipped.

In case you have trouble with the board, you can quickly switch to Slot 1 to confirm the LCD, LEDs, and similar are all functioning correctly. However there is no problem
to overwrite any of the slots, the default image can easily be copied back if you want later.

> TODO - do we want to recommend a different slot?

## Drag & Drop Programming

> TODO: Where is this bitstream located? Assume it's built by CI, or do we have a release version?

To program the Sonata bitstream:

1. Download `lowrisc_sonata_system_0.bit` from THEBESTURL.COM/lowrisc_sonata_system_0.bit
2. Select slot 2 using switch SW3 (`Bitstream`)
3. Plug in Sonata board. You should see a SONATA drive (see troubleshooting section if unsure).
4. Drag the updated `.bit` file and wait for the copy to complete (on Linux note the copy command may return immediately, so you need to wait until it's done.)
5. The board should automatically restart once the image is copied over. You should see the `FPGA Config` LED come on:

![](img/sonata-fpgaconfig.jpeg)

This indicates the FPGA configuration succeeded. This LED should stay on. You should also see the `Ibex Boot` LED come on indicating the processor core has booted.

> The `FPGA Config` LED reflects the state of the FPGA `DONE` pin. If this LED is not on your board will not work, as there is no logic (core)
> loaded, or it has become corrupted. This is true even if you are not building Sonata designs but using the board as a general-purpose FPGA
> board. See troubleshooting below if this LED does not come on, or appears to only come on briefly.

Here is the commands you'd need to do all of that, assuming Sonata was already plugged in and has been mounted at `/media/sonata`

```sh
wget THEBESTURL.COM/lowrisc_sonata_system_0.bit
cp lowrisc_sonata_system_0.bit /media/sonata/.
```

Many Linux desktop distributions will automount if you open the drive via the graphical interface, so you may find it easier to do this from your Linux desktop,
which 2024 is certainly the year of.

### Programming on Power Cycle

Once the copy completes (it can take from 15-120 seconds), you should see the device reboot and the Ibex boot LED come on as mentioned. If you unplug & replug the USB cable,
it will also reprogram the FPGA. The bitstream is stored on FLASH memory on the Sonata board.

## Troubleshooting

### Sonata Mass Storage Drive Issues

The Sonata board when plugged in should show a mass storage drive with these files:

```
LOG.TXT
OPTIONS.TXT
README.TXT
```

If the board has a RPI loader, visible because you'll see the file `INFO_UF2.txt` with the contents `Model: Raspberry Pi RP2`, you may need to reload the RP2040 as described in the [Sonata Board Updating Firmware]() section. This could
be because the `RPI Boot` button was held down when plugging in the board.

The Sonata board will print status and messages to the `LOG.TXT` which can be helpful for debugging. It should show the status of valid-looking bitstreams:

```
CRIT: FW_VER 0.1.2
INFO: No bitstream in slot 0
INFO: No bitstream in slot 1
INFO: No bitstream in slot 2
INFO: Using slot 2
INFO: No bitstream in flash @ 1400000
```

### FPGA Config Led not coming on

If the `FPGA Config` LED is not coming on, this could indicate the bitstream was designed for a different FPGA, or some other hardware issue. This should be troubleshooted with the OpenFPGALoader utility as described further down this page.

### Ibex Boot LED not coming on

If the `FPGA Config` LED is on but the `Ibex Boot` LED is not, it may be that you have programmed (or selected) a different bitstream than one that runs the CHERIoT demo. Try reloading the bitstream, and try power cycling the device.

### Device Rebooting During/After Programming

The Sonata board takes a fair amount of power (>500mA) from the USB interface, and should be connected via USB-C. Typically it is close enough to the USB 2.0 limits that it will work with the adapter most of the time, but if you are having reliability issues we recommend trying a different computer, ideally one with a USB-C port.

### FPGA Programming via USB/JTAG

If for some reason the mass storage programming isn't working, you can also use the built-in FTDI JTAG programming. This requires the setup described in [FPGA Programming](../dev/fpga-programming.md) to build openFPGALoader. Once built, you simply run:

```sh
openFPGALoader -c ft4232 sonata_top.bit
```

Note this requires the `udev` setup described in [FPGA Programming](../dev/fpga-programming.md). If you are lazy you can just run the command as root instead (not recommended, but can be helpful for troubleshooting on VMs):

```sh
sudo openFPGALoader -c ft4232 sonata_top.bit
```
You can also check if the flag `--read-register STAT` is available (newer than 0.12.2 is needed) which will print detailed information about the configuration status with a recent version of openFPGALoader. This is especially helpful if you are trying to understand why the `DONE` LED is not coming on:

```
CRC Error      No CRC error
Part Secured   0
MMCM lock      1
DCI match      1
EOS            0
GTS CFG B      0
GWE            0
GHIGH B        0
MODE           7
INIT Complete  1
INIT B         0
Release Done   0
Done           0
ID Error       ID error
DEC Error      0
XADC Over temp 0
STARTUP State  0
Reserved       0
BUS Width      x1
Reserved       8
```
