# Sonata system

Sonata is a system for evaluating the usage of [CHERIoT Ibex core](https://github.com/microsoft/cheriot-ibex) as a microcontroller for embedded, IoT and Operational Technology applications.
The system contain a number of peripherals (I2C, SPI, GPIO, PWM, UART, and DMA) and a CHERIoT enabled debug module along with the CHERIoT Ibex core itself.
It is designed for use on FPGA and specifically targets the [Sonata FPGA board](https://github.com/newaetech/sonata-pcb).

It is under active development and the first full RTL release is not yet complete.
In the meantime, please read the [architecture specfication](https://lowrisc.org/sonata-system/doc/architecture/intro.html).

Sonata is part of the [Sunburst Project](https://www.sunburst-project.org) funded by [UKRI](https://www.ukri.org/) / [DSbD](https://www.dsbd.tech/).

## Building Documentation

The documentation uses [mdBook](https://rust-lang.github.io/mdBook/) see the [installation guide](https://rust-lang.github.io/mdBook/guide/installation.html) for further details on installation.

Once mdBook is installed the documentation can be built and viewed with:

```bash
mdbook serve --open
```

## License

Unless otherwise noted, everything in the repository is covered by the [Apache License](https://www.apache.org/licenses/LICENSE-2.0.html), Version 2.0. See the [LICENSE](https://github.com/lowRISC/sonata-system/blob/main/LICENSE) file for more information on licensing.
