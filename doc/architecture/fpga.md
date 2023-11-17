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

We also need to modify CHERIoT Ibex to output the CHERIoT mode and exception codes so that these can be hardwired to the appropriate LEDs.

## Memory layout

For all registers in this section, the functionality is mapped onto the least significant bits of registers and each register is 32 bits wide.

| Base address | Size    | Functionality  |
|--------------|---------|----------------|
| 0x0010_0000  |  64 KiB | Internal SRAM  |
| 0x1a11_0000  |   4 KiB | [Debug module] |
| 0x8000_0000  |   4 KiB | [GPIO]         |
| 0x8000_1000  |   4 KiB | [UART]         |
| 0x8000_2000  |   4 KiB | [Timer]        |
| 0x8000_3000  |   4 KiB | [I2C host]     |
| 0x8000_4000  |   4 KiB | [SPI host]     |
| 0x8000_5000  |   4 KiB | [Ethernet]     |
| 0x8000_6000  |   4 KiB | [HyperRAM]     |
| 0x8000_7000  |   4 KiB | [DMA]          |
| 0x8000_8000  |   4 KiB | [ADC]          |

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

## Clocking infrastructure

The whole system is driven by the same clock with the exception of the HyperRAM controller.
Optionally the HyperRAM controller can be clocked higher than the rest of the chip.
To accommodate this, we introduce a synchronization interface with primitive FIFOs.
