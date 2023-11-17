# Physical board architecture

This section focusses on what needs to be physically present on the board and explicitly leaves the configuration of the FPGA and the software for later sections.

## Configuration

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

## Peripherals

The connectable requirement means that we need to introduce common peripherals on the board.
After consultation with the community, we settled on the following list:
- Ethernet
- RS232
- RS485
- MicroSD card

## Headers

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

## Debug

Together with being easy to use, it would be nice to have one USB connector that can power the board as well as debug using JTAG and UART.
This means that users only have to connect one cable.

Besides the JTAG over USB, it is also good to provide external JTAG and UART headers to enable users to use different setups if they need to.

## User interface

The user interface is where we address the CHERI-visible and interactable requirements.
In terms of user input, we have:
- DIP switches
- Button
- Joystick

In terms of output, we have:
- LEDs
- LCD screen
- CHERI-specific capability exception LEDs

## Affordable

In order to meet the affordability aspect, we choose a low-end FPGA to reduce the costs.
We choose a Xilinx Artix 7 FPGA because it has a typical amount of memory for embedded use-cases while being able to clock the design higher than similarly priced alternatives and being supported by many tools.
