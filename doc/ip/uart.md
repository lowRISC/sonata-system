# Universal asynchronous receiver/transmitter (UART)

The Sonata system uses the OpenTitan UART.
The version of the IP that is vendored in has documentation that you can find [here](https://github.com/lowRISC/opentitan/tree/a78922f14a8cc20c7ee569f322a04626f2ac6127/hw/ip/uart/doc).
You can find [the register definitions here](https://github.com/lowRISC/opentitan/blob/a78922f14a8cc20c7ee569f322a04626f2ac6127/hw/ip/uart/doc/registers.md).

There are multiple UART instances in Sonata to connect to any of the following targets:
- USB
- RS-232
- mikroBUS
- Arduino shield
- Raspberry Pi hat

By default, Sonata includes 3 UART blocks, each with an offset of `0x1000` from each other.
