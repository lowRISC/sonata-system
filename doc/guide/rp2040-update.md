# Reloading the RP2040 USB Controller

Before plugging in your Sonata board, hold down the SW9 labelled "RP2040 Boot", and while holding this button plug your board into your laptop using the Main USB.

A drive called 'RPI-RP2' should pop up on your computer and drag `rpi_rp2_vX.Y.uf2` into it.
This drive should automatically dismount once the file is transferred and remount as 'SONATA'.

## Downloading the `rpi_rp2_vX.Y.uf2` file

Currently the RP2040 firmware is available from the [Sonata Systems release page](https://github.com/lowRISC/sonata-system/releases), which ensures your RP2040 firmware matches the Sonata FPGA and firmware expectations.

The source & latest release for the RP2040 are also found on the [Sonata RP2040 repo](https://github.com/newaetech/sonata-rp2040/releases).
