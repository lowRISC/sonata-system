# Universal asynchronous receiver/transmitter (UART)

The Sonata system uses [the OpenTitan UART](https://opentitan.org/book/hw/ip/uart/index.html).
You can find [the register definitions here](https://opentitan.org/book/hw/ip/uart/doc/registers.html).

There are multiple UART instances in Sonata to connect to any of the following targets:
- USB
- RS-232
- mikroBUS
- Arduino shield
- Raspberry Pi hat

By default, Sonata includes 3 UART blocks, each with an offset of `0x100` from each other.
