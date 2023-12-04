# Pin multiplexer

This allows software to dynamically switch FPGA pins between input and output as well as reassign them for SPI, I2C, UART, etc.
The block also allows pad control.
An example use-case is changing whether an LED is driven by the GPIO or by the PWM.

We will use [the pin multiplexer from OpenTitan](https://opentitan.org/book/hw/ip/pinmux/index.html), but then parameterized for Sonata.
More details on how this changes the register map will be provided in the future.

In general, we expect each pin can be connected to a limited subset of blocks.
The output of GPIO and PWM should only be able to control one pin, while blocks like UART, SPI and I2C should have multiple pins that they can connect to.
The exact mapping of which pin can be driven by which hardware IP block will be provided in the future.
