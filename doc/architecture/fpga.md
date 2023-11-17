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
- SPI for flash
- Ethernet
- GPIO for buttons and LEDs

There might be other IP blocks necessary for interacting with headers such as an analogue to digital converter.

We also need to modify CHERIoT Ibex to output the CHERIoT mode and exception codes so that these can be hardwired to the appropriate LEDs.

## Memory layout

For all registers in this section, the functionality is mapped onto the least significant bits of registers and each register is 32 bits wide.

| Base address | Size    | Functionality |
|--------------|---------|---------------|
| 0x0010_0000  |  64 KiB | Internal SRAM |
| 0x1a11_0000  |   4 KiB | Debug module  |
| 0x8000_0000  |   4 KiB | GPIO          |
| 0x8000_1000  |   4 KiB | UART          |
| 0x8000_2000  |   4 KiB | Timer         |
| 0x8000_3000  |   4 KiB | I2C host      |
| 0x8000_4000  |   4 KiB | SPI host      |
| 0x8000_5000  |   4 KiB | Ethernet      |

### Debug module

### GPIO

| Offset | Register |
|--------|----------|
| 0x00   | Output   |
| 0x04   | Input    |
| 0x08   | Debounced input |

The output register displays the specified value onto the boards output:

| Bit offset | Description |
|------------|-------------|
| 7-0        | LEDs        |

Both input registers:

| Bit offset | Description |
|------------|-------------|
| 13-9       | Joystick (left, down, up, right, press) |
| 8          | Button |
| 7-0        | DIP switches |

### UART

| Offset | Register |
|--------|----------|
| 0x00   | Receive |
| 0x04   | Transmit |
| 0x08   | Status |

Receive data:

| Bit offset | Description |
|------------|-------------|
| 7-0        | Received byte |

Transmit data:

| Bit offset | Description |
|------------|-------------|
| 7-0        | Byte to write to UART |

Status:

| Bit offset | Description |
|------------|-------------|
| 1          | Transmit FIFO full |
| 0          | Receive FIFO empty |

### I2C host

| Offset | Register |
|--------|----------|
| 0x00   | Reserved |
| 0x04   | Status   |
| 0x08   | Read data |
| 0x0C   | Format data |
| 0x10   | FIFO control |
| 0x14   | FIFO status |
| 0x18   | Reserved |
| 0x1C   | Reserved |
| 0x20   | Timing low and high |
| 0x24   | Timing fall and rise |
| 0x28   | Timing start and repeated start |
| 0x2C   | Timing hold and setup |
| 0x30   | Timing buffer and stop |
| 0x34   | Stretch timeout |
| 0x38   | Reserved |
| 0x3C   | Reserved |
| 0x40   | Reserved |
| 0x44   | Clock timeout |

Status:

| Bit offset  | Description |
|-------------|-------------|
| 9           | Acquire FIFO empty |
| 8           | Transmit FIFO empty |
| 7           | Acquire FIFO full |
| 6           | Transmit FIFO full |
| 5           | Receive FIFO empty |
| 4           | Reserved |
| 3           | Host idle |
| 2           | Format FIFO empty |
| 1           | Receive FIFO full |
| 0           | Format FIFO full |

Read data:

| Bit offset  | Description |
|-------------|-------------|
| 7-0         | Read data   |

Format data:

| Bit offset  | Description |
|-------------|-------------|
| 12          | Don't acknowledge current byte |
| 11          | Continue read after last byte |
| 10          | Read format byte number of bytes (256 if format byte is zero) |
| 9           | Issue stop after this operation |
| 8           | Issue start before byte |
| 7-0         | Format data |

FIFO control:

| Bit offset  | Description |
|-------------|-------------|
| 8-2         | Reserved    |
| 1           | Format FIFO reset |
| 0           | Receive FIFO reset |

FIFO status:

| Bit offset  | Description |
|-------------|-------------|
| 30-24       | Reserved    |
| 22-16       | Receive FIFO fill level |
| 14-8        | Reserved |
| 6-0         | Format FIFO fill level |

Timing low and high:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold SCL low |
| 15-0        | Time to hold SCL high |

Timing fall and rise:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Anticipated bus fall time |
| 15-0        | Anticipated bus rise time |

Timing start and repeated start:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold start signals |
| 15-0        | Setup time for repeated start signals |

Timing hold and setup:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold data and acknowledge bits |
| 15-0        | Setup time for data and acknowledge bits |

Timing buffer and stop:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time between each stop signal and following start signal |
| 15-0        | Setup time for stop signals |

Stretch timeout control:

| Bit offset  | Description |
|-------------|-------------|
| 31          | Enable timeout |
| 30-0        | Timeout value |

Clock timeout control:

| Bit offset | Description |
|------------|-------------|
| 31-0       | Host clock generation timeout value |

### SPI host

| Offset | Register |
|--------|----------|
| 0x00   | Transmit |
| 0x04   | Status   |

The transmit register is used to write data to the SPI one byte at a time:

| Bit offset | Description |
|------------|-------------|
| 7-0        | Write data  |

Status register:

| Bit offset  | Description |
|-------------|-------------|
| 1           | FIFO empty  |
| 0           | FIFO full   |

### Ethernet
