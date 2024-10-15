# Software Programming

There are four ways of programming the software:

1. You can use the flash storage on the Sonata board. This does not require special tools and allows an image to come
   online automatically at boot.

2. You can use OpenOCD to program the image onto RAM and then run this image. This is typically used during development.

3. You can use the CHERIoT serial bootloader. This loads the image into RAM on the CHERIoT system and then runs the image.

4. You can 'splice' the software into the FPGA image. This provides a single 'bitstream' including both hardware definition and software.
  This can be helpful for making system images that come alive as soon as possible after boot.

## Flash Programming

## OpenOCD/JTAG Programming
### Programing the testsuite using openocd
The script `mem_helper.sh` can be used to load any elf via openocd/JTAG.
```sh
./util/mem_helper.sh load_program -e sw/cheri/build/tests/test_runner
```
Open the uart in order to check the test output.
```sh
picocom /dev/ttyUSB2 -b 921600 --imap=lfcrlf
```
Note: Test runs quite quickly, so you may need to press the Reset button on the board to run it again and see the logs.

### Programing the testsuite using nix
```sh
nix run .#fpga-test /dev/ttyUSB2
```

## Serial Bootloader

> Will this still be supported?

## Splicing into FPGA Image
