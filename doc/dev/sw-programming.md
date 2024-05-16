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

## Serial Bootloader

> Will this still be supported?

## Splicing into FPGA Image