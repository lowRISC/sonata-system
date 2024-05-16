# Sonata system

Sonata is a system for evaluating the usage of [CHERIoT Ibex core](https://github.com/microsoft/cheriot-ibex) as a microcontroller for embedded, IoT and Operational Technology applications.
The system contain a number of peripherals (I2C, SPI, GPIO, PWM, UART, and DMA) and a CHERIoT enabled debug module along with the CHERIoT Ibex core itself.

It is designed for use on FPGA and specifically targets the [Sonata FPGA board](https://github.com/newaetech/sonata-pcb), but as the entire design (from example PCB to software) is open-source it can be run on any similar system.

This project is designed to look like a normal microcontroller in terms of usability, including SDK, examples, and normal capabilities such as debuggers. But underneath that the [CHERIoT](https://www.microsoft.com/en-us/research/publication/cheriot-rethinking-security-for-low-cost-embedded-systems/) capabilities provides a high level of "default security" that simplifies designing embedded systems in a secure manner. You can see the [complete documentation](https://lowrisc.org/sonata-system/) for the project, but note it is under active development so substantial improvements are to be made.

As this project is under active development the first full RTL release is not yet complete. We plan to release an updated image, together with instructions for uploading it to the board over USB, before the May 29th 2024 Hackathon event. This image will allow running CHERIoT-RTOS and will allow for general purpose software development.

Sonata is part of the [Sunburst Project](https://www.sunburst-project.org) funded by [UKRI](https://www.ukri.org/) / [DSbD](https://www.dsbd.tech/) under grant number 107540.

## Getting Started

![](doc/img/sonata-full.jpeg)

If you have a Sonata board, you can jump to the [Getting Started](doc/guide/getting-started.md) guide. This will walk you through plugging in the board, building example software, and programming the software. For more advanced usage, you can see a [Reference Manual](doc/dev/ref-manual.md) similar to what a normal microcontroller reference manual (peripherals, features, etc) and then see the [FPGA development](doc/dev/fpga-programming.md) flow if you wish to modify the soft-core itself

You can also work with a simulated environment, see the simulator page. This simulates the entire soft-core in Verilator, allow you to develop both hardware (the FPGA) and software (running code) programs.

## Documentation Intro

This documentation is built using [mdBook](https://rust-lang.github.io/mdBook/). If you are reading this file in GitHub, you should instead see the pre-built documentation on the [lowRISC Website](https://lowrisc.org/sonata-system/) which includes the full documentation.

If you'd like to build a copy of the documentation locally, see the [Building Documentation](doc/dev/building-doc.md) page.

## License

Unless otherwise noted, everything in the repository is covered by the [Apache License](https://www.apache.org/licenses/LICENSE-2.0.html), Version 2.0. See the [LICENSE](https://github.com/lowRISC/sonata-system/blob/main/LICENSE) file for more information on licensing.
