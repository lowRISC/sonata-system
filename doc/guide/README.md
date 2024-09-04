# Getting started with the Sonata board

This guide helps you to get started with the Sonata board by building code with CHERIoT technology enabled. One more advanced feature of Sonata is you can adjust the number
and type of peripherals included. This is described in the section [FPGA Development](../dev/fpga-development.md), for this getting started guide you will use one of the
default setups.

> The Sonata board is a prototype board and is under active development.
> This documentation is in the process of being updated.
> Some parts of the documentation may be out of date or otherwise confusing because of this.
> This getting started material and the documentation in the [sonata-software](https://lowrisc.github.io/sonata-software/doc/getting-started.html) repository provides the best material for new users to focus on.

## Getting started steps

To get started with your Sonata board, there are three steps you'll need to do. First, head to
the [Sonata System Release Page](https://github.com/lowRISC/sonata-system/releases/) where you'll find the latest releases.

1. [Program the RP2040 With the latest](rp2040-update.md) firmware to get any bug fixes by entering bootloader mode & dragging the `rpi_rp2_v0.XX.uf2`.
2. [Get the latest FPGA image](fpga-update.md) that corresponds with the software you are building. This requires you to just drag the new `sonata_vXX.bit.slot1.uf2` file into the 'SONATA' drive that comes up when you plug in the board to your computer.
3. [Build the example code](building-examples.md) and download to the soft-core you loaded in step 1.

Follow along each of the following sections to complete these tasks.
