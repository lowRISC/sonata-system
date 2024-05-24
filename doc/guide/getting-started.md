# Getting started with Sonata Board

This guide helps you to get started with the Sonata board by building code with CHERIoT technology enabled. One more advanced feature of Sonata is you can adjust the number
and type of peripherals included. This is described in the section [FPGA Development](../dev/fpga-development.md), for this getting started guide you will use one of the
default setups.

> The Sonata board is a prototype board and is under active development.
> This documentation is in the process of being updated.
> We will be improving the getting started guide soon to allow for an easier way with developing against the boards.

## Getting Started Steps

To get started with your Sonata board, there are three steps you'll need to do. First, head to
the [Sonata System Release Page](https://github.com/lowRISC/sonata-system/releases/) where you'll find the latest releases.

0. [Program the RP2040 With the Latest](rp2040-update.md) firmware to get any bug fixes by entering bootloader mode & dragging the `rpi_rp2_v0.XX.uf2`.
1. [Get the latest FPGA image](fpga-update.md) that corresponds with the software you are building. This requires you to just drag the new `.bit` file onto the SONATA drive that comes up when you plug in the board to your computer.
2. [Install the software toolchain](toolchain-setup.md).
3. [Build the example code](building-examples.md) and download to the soft-core you loaded in step 1.

Follow along each of the following sections to complete these tasks.
