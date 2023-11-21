# FPGA configuration architecture

The FPGA configuration is the part of the architecture that is programmed into the FPGA.
This is written in a hardware description language.
Although it is considered hardware it can be reprogrammed because of the FPGA, which is why it has a separate section from the physical board design.

## Interoperate

For the interoperable requirement, we need to make sure our hardware design can interact with that of OpenTitan Earl Grey.
Since OpenTitan Earl Grey uses a TileLink Uncached Lightweigh (TL-UL) bus, we use the same in the Sonata system to ease designing a bridge interface.

Another part of the Sonata system is an finite state machine (FSM) that controls the boot flow of CHERIoT Ibex.
Initially this FSM will control how the CHERIoT Ibex boots from ROM.
The boot control interface that this FSM uses should be designed in such a way that it can eventually be controlled and driven by OpenTitan Earl Grey.

## Hardware IP blocks
To support all the peripherals that are on the FPGA boards, we need corresponding hardware IP blocks for Ibex to be able to interact with them:
- I2C for QWIIC
- SPI for the LCD screen
- Ethernet
- GPIO for buttons and LEDs
- HyperRAM controller

There might be other IP blocks necessary for interacting with headers such as an analogue to digital converter.

We also need some modifications to CHERIoT Ibex, which are detailed in [its own page](../ip/ibex.md).

## Memory layout

For all registers in this section, the functionality is mapped onto the least significant bits of registers and each register is 32 bits wide.

| Base address | Size    | Functionality  |
|--------------|---------|----------------|
| 0x0010_0000  | 128 KiB | Internal SRAM  |
| 0x200f_e000  |  16 KiB | Revocation tags |
| 0x8000_0000  |   4 KiB | [GPIO]         |
| 0x8000_1000  |   4 KiB | [UART]         |
| 0x8000_2000  |   4 KiB | [Timer]        |
| 0x8000_3000  |   4 KiB | [I2C host]     |
| 0x8000_4000  |   4 KiB | [SPI host]     |
| 0x8000_5000  |   4 KiB | [Ethernet]     |
| 0x8000_6000  |   4 KiB | [HyperRAM]     |
| 0x8000_7000  |   4 KiB | [DMA]          |
| 0x8000_8000  |   4 KiB | [ADC]          |
| 0x8000_9000  |   4 KiB | [PWM]          |
| 0xA000_0000  |  64 MiB | [PLIC]         |
| 0xF000_0000  |   4 KiB | [Debug module] |

[Debug module]: ../ip/dm.md
[GPIO]: ../ip/gpio.md
[UART]: ../ip/uart.md
[Timer]: ../ip/timer.md
[I2C host]: ../ip/i2c.md
[SPI host]: ../ip/spi.md
[Ethernet]: ../ip/eth.md
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

We have a few different types of memory in the Sonata system: FPGA SRAM, HyerRAM and flash.
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
This should be fine as CHERI capabilities are derived an manipulated at runtime, but does require toolchain changes to LLVM and corresponding RTOS (see [software architecture](software.md)).

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
