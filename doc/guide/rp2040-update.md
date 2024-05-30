# Reloading the RP2040 USB Controller

*We have seen some issues with the standard FPGA loading flow using the RP2040.
Until these are fixed we recommend using a special version of the RP2040 firmware that has the FPGA's bitstream pre-built into it.
This is named `tmp_rpi_rp2_part1_v0.2.uf2` in the [current release](https://github.com/lowRISC/sonata-system/releases/tag/v0.2).
This will be used in place of the `rpi_rp2_v0.X.uf2` file referred to below.*

Before plugging in your Sonata board, hold down the SW9 labelled "RP2040 Boot", and while holding this button plug your board into your laptop using the Main USB.

A drive called RPI-RP2 should pop up on your computer and drag `rpi_rp2_v0.X.uf2` into it.
This drive should automatically dismount once the file is transferred and remount as SONATA.

## Downloading the `rpi_rp2_v.X.uf2` file

Currently the RP2040 firmware is available from the [Sonata Systems](https://github.com/lowRISC/sonata-system/releases) releases, which ensures your RP2040 firmware matches the Sonata FPGA and firmware expectations.

The source & latest release for the RP2040 are also found on the [Sonata RP2040 repo](https://github.com/newaetech/sonata-rp2040/releases).
