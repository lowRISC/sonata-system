# Platform-level interrupt controller (PLIC)

The PLIC is a RISC-V interrupt controller and specifically we are using [OpenTitan's interrupt controller](https://opentitan.org/book/hw/top_earlgrey/ip_autogen/rv_plic/index.html).
Please find more details in [the official specification](https://github.com/riscv/riscv-plic-spec/releases/download/1.0.0/riscv-plic-1.0.0.pdf).
Part of these details are is the memory map, which shows all registers from the `base` until `base + 0x3FFFFFC`.
Not all of these registers are mapped, most importantly we only have one core, so only one context (context 0).

In terms of interrupts, the exact mapping of sources to interrupts will be published here at a later point.
There are a number of considerations.

- The following blocks have interrupts: timer, UART, I2C, SPI, ...
- There are multiple UART, I2C and SPI blocks in the system.
- Some blocks have multiple interrupts.
