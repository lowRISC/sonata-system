# Updating the Sonata System

To get started with your Sonata board, there are three levels of hardware/software designs you'll need to be able to load onto the board.

First, head to the [Sonata System Release Page](https://github.com/lowRISC/sonata-system/releases/) where you'll find the latest releases.
Each release includes an RP2040 firmware image and FPGA bitstream. Once you have downloaded these, proceed with the following tasks:

1. [Program the RP2040 With the latest](rp2040-update.md) firmware to get any bug fixes by entering bootloader mode & dragging the `rpi_rp2_vX.Y.uf2`.
2. [Get the latest FPGA image](fpga-update.md) that corresponds with the software you are building. This requires you to just drag the new `sonata-vX.Y.bit.slot1.uf2` file into the 'SONATA' drive that comes up when you plug in the board to your computer.
3. [Build the example code](building-examples.md) and download to the soft-core you loaded in step 1.

Follow along each of the following sections to complete these tasks.
