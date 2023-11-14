# Sonata architecture specification

The Sonata architecture comprises a number of components:
- Physical board architecture:
  This is the board that hosts the FPGA (field programmable gate array), all other components, headers and interfaces.
- FPGA configuration architecture:
  It defines everything we are using a hardware description language for.
  This includes hardware IP blocks and bus architecture.
- Software architecture:
  This includes toolchain, operating system and applications.

Before we go into the architecture of the system, it is good to understand the use cases that we are envisioning, so that we can derive our architectural requirements from that.

## Use cases and requirements

The Sonata is meant to be used by academics and industry users who are interested in experimenting with CHERIoT in embedded and IoT applications.
This is the main reason why we are building a custom FPGA board so that we can make the platform easily *usable*.
We also need the board to be as *affordable* as possible while still being performant and usable.

This ease of use comes in handy for classroom and demonstration use cases, for which we think this board will come in quite handy.
In the classroom, it is also pertinent that we have a *debuggable* system.

Because we are focussing on embedded and IoT applications, we need to ensure connectivity.
This includes being *connectable* to standard peripherals as well as being *extendable* with functionality required for niche use-cases.

The other major benefit of creating a custom board is that we can highlight CHERIoT specific features.
We envision users of this board to want to show off and experiment with the new CHERIoT technology.
If they are using the board as a demonstrator they will want to show off the prowess of CHERI, and if they are experimenting with the board they will want *CHERI to be visible*.

Like most physical boards, it is nice for it to be as interactive as possible.
This means being able to accept user input and respond to them visibly and in an *interactable* way.

Beyond the physical Sonata board, we expect the soft design that is used to configure the FPGA to also be *integratable* into a bigger design.
For example, we envision it being connected to OpenTitan Earl Greg through a bridge interface.

In short, these are our general requirements:
- Usable
- Affordable
- Debuggable
- Connectable
- Extendable
- CHERI visible
- Interactable
- Integratable

## Physical board architecture

This section focusses on what needs to be physically present on the board and explicitly leaves the configuration of the FPGA and the software for later sections.

### Configuration

The usable requirement makes it worth thinking well about configuration.
We want to provide multiple bitstreams and software images that can be switched between on the board without having to reprogram it.
This is why we have a switch for bitstream and a switch for software images.

For example, we could have a CHERI and non-CHERI bitstream both available on the board.
For software, we can switch between demo applications, for example CHERI compartmentalization versus CHERI exceptions.
Introducing these physical switches also fulfills the interactable requirement.

To make multiple bitstreams available, we introduce a USB connector that looks like mass storage to a user, where multiple bitstreams can be stored and changed without hassle.
There wil be and RP2040 on the board to manage these configurations.

Also part of this usability requirement is to have enough memory to store the bitstreams and software.
We introduce two separate flash chips for these purposes and a HyperRAM chip.

### Peripherals

The connectable requirement means that we need to introduce common peripherals on the board.
After consultation with the community, we settled on the following list:
- Ethernet
- RS232
- RS485
- MicroSD card

### Headers

For both the connectable and extendable requirements, we provide a number of headers so that custom functionality can be added:
- Raspberry Pi header
- Arduino shield
- MicroBus click
- Sparkfun QWIIC
- PMOD
- 15-pin R/A header

Due to space limitation the headers for the Raspberry Pi, Arduino shield and MicroBus click cannot be used simultaneously.
The same is true for the 2 PMODs and the R/A header.
We don't expect this to be a problem as most applications should only need to use one expansion board.

### Debug

Together with being easy to use, it would be nice to have one USB connector that can power the board as well as debug using JTAG and UART.
This means that users only have to connect one cable.

Besides the JTAG over USB, it is also good to provide external JTAG and UART headers to enable users to use different setups if they need to.

### User interface

The user interface is where we address the CHERI-visible and interactable requirements.
In terms of user input, we have:
- DIP switches
- Button
- Joystick

In terms of output, we have:
- LEDs
- LCD screen
- CHERI-specific capability exception LEDs

### Affordable

In order to meet the affordability aspect, we choose a low-end FPGA to reduce the costs.
We choose a Xilinx Artix 7 FPGA because it has a typical amount of memory for embedded use-cases while being able to clock the design higher than similarly priced alternatives and being supported by many tools.

## FPGA configuration architecture

The FPGA configuration is the part of the architecture that is programmed into the FPGA.
This is written in a hardware description language.
Although it is considered hardware it can be reprogrammed because of the FPGA, which is why it has a separate section from the physical board design.

### Interoperate

For the interoperable requirement, we need to make sure our hardware design can interact with that of OpenTitan Earl Grey.
Since OpenTitan Earl Grey uses a TileLink Uncached Lightweigh (TL-UL) bus, we use the same in the Sonata system to ease designing a bridge interface.

Another part of the Sonata system is an finite state machine (FSM) that controls the boot flow of CHERIoT Ibex.
Initially this FSM will control how the CHERIoT Ibex boots from ROM.
The boot control interface that this FSM uses should be designed in such a way that it can eventually be controlled and driven by OpenTitan Earl Grey.

### Hardware IP blocks
To support all the peripherals that are on the FPGA boards, we need corresponding hardware IP blocks for Ibex to be able to interact with them:
- I2C for QWIIC
- SPI for flash
- Ethernet
- GPIO for buttons and LEDs

There might be other IP blocks necessary for interacting with headers such as an analogue to digital converter.

We also need to modify CHERIoT Ibex to output the CHERIoT mode and exception codes so that these can be hardwired to the appropriate LEDs.

### Memory layout

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

#### GPIO

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

#### UART

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

#### I2C host

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

#### SPI host

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

## Software architecture

In order to make the Sonata system usable, we need to facilitate running software on it.
We need a compiler toolchain to bootstrap this whole system.

### Toolchain

The toolchain will build on top of the work already done on CHERIoT, which uses a [fork of LLVM](https://github.com/CHERIoT-Platform/llvm-project).
We do not envision needing to change anything here because we are taking the CHERIoT ISA as it is and not making any functional changes to CHERIoT Ibex.

### Applications

Especially for the usable and CHERI-visible requirements, it is important that we have a set of demonstration applications.
One demonstration application is cycling through each of the CHERIoT exception types with code snippets showing what went wrong.
We can show what happens when CHERI is enabled and when it is disabled.
We will provide at least some of these applications in bare-metal mode where they do not need an operating system.

### Operating system

The [CHERIoT RTOS work](https://github.com/microsoft/cheriot-rtos) will need some reworking in terms of memory layout and drivers to work on the Sonata system.
We envision this not being particularly difficult for someone with software knowledge, but will initially be left out of scope of our work.

## Conclusion

The Sonata system focuses on being:
- Usable
- Connectable
- Extendable
- Debuggable
- CHERI visible
- Interactable
- Affordable
- Integratable

The goal of the Sunburst project is to get CHERIoT hardware into the hands of engineers and Sonata is a crucial step in that process.
