# Reference Manual for Sonata Core

In a classical microcontroller, you would have a core along with the peripherals around the core. On the Sonata system this is all part
of an open-source FPGA design, which allows you to modify the core to add new features (and for us to add updates to your core without
needing you to desolder your main IC!).

This also means you can customize your design. You may want to have a different number of UARTs or SPI blocks for example. This document
describes the base configuration, but you can 

The FPGA image is parameterizable to enable custom setups.
It should be easy, for example, to change the number of UART, SPI and I2C instances.
We will provide pre-built images for common configurations.

## Interoperate

For the interoperable requirement, we need to make sure our hardware design can interact with that of OpenTitan Earl Grey.
Since OpenTitan Earl Grey uses a TileLink Uncached Lightweight (TL-UL) bus, we use the same in the Sonata system to ease designing a bridge interface.

## Hardware IP blocks

To support all the peripherals that are on the FPGA boards, we need corresponding hardware IP blocks for Ibex to be able to interact with them:
- I2C for QWIIC
- SPI for the LCD screen and ethernet
- GPIO for buttons and LEDs
- HyperRAM controller

There might be other IP blocks necessary for interacting with headers such as an analogue to digital converter.
We also need some modifications to CHERIoT Ibex, which are detailed in [its own page](../ip/ibex.md).

Wherever possible, we reuse existing, high-quality, open-source hardware IP blocks that are fit for purpose.

## Memory layout

For all registers in this section, the functionality is mapped onto the least significant bits of registers and each register is 32 bits wide.

| Base address | Size    | Functionality    |
|--------------|---------|------------------|
| 0x0010_0000  | 128 KiB | Internal SRAM    |
| 0x3000_0000  |  16 KiB | Revocation tags  |
| 0x4000_0000  |   1 MiB | Tagged RAM       |
| 0x4010_0000  |   7 MiB | Untagged RAM     |
| 0x8000_0000  |   4 KiB | [GPIO][]         |
| 0x8000_1000  |   4 KiB | [PWM][]          |
| 0x8000_2000  |   4 KiB | [DMA][]          |
| 0x8000_3000  |   4 KiB | [HyperRAM][]     |
| 0x8000_4000  |   4 KiB | [ADC][]          |
| 0x8000_5000  |   4 KiB | [Pinmux][]       |
| 0x8004_0000  |  64 KiB | [Timer][]        |
| 0x8010_0000  |   1 MiB | [UART][]         |
| 0x8020_0000  |   1 MiB | [I2C host][]     |
| 0x8030_0000  |   1 MiB | [SPI host][]     |
| 0x8040_0000  |   1 MiB | [USB device][]   |
| 0x8800_0000  |  64 MiB | [PLIC][]         |
| 0xB000_0000  |   4 KiB | [Debug module][] |

[Debug module]: ../ip/dm.md
[GPIO]: ../ip/gpio.md
[UART]: ../ip/uart.md
[Timer]: ../ip/timer.md
[I2C host]: ../ip/i2c.md
[SPI host]: ../ip/spi.md
[USB device]: ../ip/usb.md
[HyperRAM]: ../ip/ram.md
[DMA]: ../ip/dma.md
[ADC]: ../ip/adc.md
[PWM]: ../ip/pwm.md
[PLIC]: ../ip/plic.md
[Pinmux]: ../ip/pinmux.md

## Clocking infrastructure

The whole system is driven by the same clock with the exception of the HyperRAM controller.
Optionally the HyperRAM controller can be clocked higher than the rest of the chip.
To accommodate this, we introduce a synchronization interface with primitive FIFOs.

## Memory architecture

We have a few different types of memory in the Sonata system: FPGA SRAM, HyperRAM and flash.
With CHERI we need to think about capability tags and revocation tags.
Any memory that needs to contain capabilities must have one capability tag per 32 bits.
Any memory that needs to be revocable must have one revocation tag per 32 bits.

### Capability tags

All capability tags live in SRAM.
All SRAM that is allocated for code and data will have corresponding capability tags.
Any data stored to HyperRAM and flash are not expected to be tagged.
Since capability tags are out of band information and do not need to be memory mapped, we can store these within the error correction bits that are available on the FPGA's memory.

We envision that code can live in HyperRAM with an instruction cache for improved performance.
However, this does require code to be able to live in untagged memory.
This should be fine as CHERI capabilities are derived and manipulated at runtime, but does require toolchain changes to LLVM and the corresponding RTOS (see [software architecture](software.md)).

### Revocation tags

Revocation tags are essential in providing temporal memory safety in CHERI.
This only covers a subset of memory that is likely to be used by the heap.
Setting the revocation bit effectively stops any capability with that base address from being loaded from memory.
This is a temporary step as the revocation engine scans through memory to invalidate all capabilities to this address.
Once the complete memory is scanned, the revocation bit can be unset and the memory can be reused.

In Sonata, the revocation tags only cover a subset of mapped memory.
They should apply to memory regions that are most likely to be used as heap, it is likely this will cover all of internal SRAM and some of HyperRAM.
Unlike capability tags, revocation tags need to be memory mapped so the memory allocator can manipulate them.

In CHERIoT Ibex the size of memory allocated for this is defined by `TSMapSize` which indicates how many 32-bit words can be used for revocation bits.
The default value for this is `1,024`, which corresponds to 8 KiB.
In CHERIoT Safe the size of the revocation tag memory is 16 KiB.

### List of SRAM blocks

Here's a list of blocks by size that we need to allocate in SRAM.
The XC7A35T has 100 blocks of 18 kilobit block RAM, [see datasheet](https://docs.xilinx.com/v/u/en-US/ds180_7Series_Overview).
In total that gives use 225 KiB of block RAM, but we may not efficiently map onto 18 kilobit blocks and thus lose some memory.
The block RAM usage in the table below was calculated using Vivado 2023.2's block memory generator.

| Type                   | Size    | Width | Depth  | RAM Blocks |
|------------------------|---------|-------|--------|------------|
| Internal memory        | 128 KiB |    33 | 32,768 |         60 |
| Revocation tags        |  16 KiB |    32 |  4,096 |          8 |
| RAM capability tags    |  32 KiB |    32 |  8,192 |         15 |
| Instruction cache data |   4 KiB |    64 |    512 |          2 |
| Instruction cache tags |   1 KiB |    22 |    512 |          1 |
| Total                  | 181 KiB |       |        |         86 |
| Available              | 225 KiB |       |        |        100 |
