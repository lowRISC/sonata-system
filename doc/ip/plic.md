# Platform-level interrupt controller (PLIC)

The PLIC is a RISC-V interrupt controller and specifically we are using [OpenTitan's interrupt controller](https://opentitan.org/book/hw/top_earlgrey/ip_autogen/rv_plic/index.html).
Please find more details in [the official specification](https://github.com/riscv/riscv-plic-spec/releases/download/1.0.0/riscv-plic-1.0.0.pdf).
Part of these details are is the memory map, which shows all registers from the `base` until `base + 0x3FFFFFC`.
Not all of these registers are mapped, most importantly we only have one core, so only one context (context 0).

The following table shows the current list of interrupts that are fed through the PLIC.
For more detailed descriptions of what each interrupt means, please refer to the documentation of each individual hardware IP block.
For most blocks, interrupts are multiplexed - all hardware interrupt sources are presented as a single interrupt to the PLIC.
The relevant software `INTR_STATE` registers should be queried by software to determine the interrupt cause.

| Number | Block      | Interrupt description |
|--------|------------|-----------------------|
| 0      | None       | Tied to zero
| 1      | Revoker    | Hardware revoker sweep complete
| 2      | Ethernet   | Interrupt from external SPI ethernet chip (KSZ8851SNLI-TR)
| 3      | USB Dev    | Shared USB Device interrupt
| 4      | GPIO       | Shared GPIO interrupt
| 5-7    | None       | Reserved
| 8      | UART 0     | Shared interrupt for UART0
| 9      | UART 1     | Shared interrupt for UART1
| 10     | UART 2     | Shared interrupt for UART2
| 11-15  | None       | Reserved
| 16     | I2C 0      | Shared interrupt for I2C0
| 17     | I2C 1      | Shared interrupt for I2C1
| 18-23  | None       | Reserved
| 24     | SPI LCD    | Shared interrupt for the LCD SPI
| 25     | SPI Ethmac | Shared interrupt for the Ethernet SPI
| 26     | SPI 0      | Shared interrupt for SPI0
| 27     | SPI 1      | Shared interrupt for SPI1
| 28     | SPI 2      | Shared interrupt for SPI2
| 29-31  | None       | Reserved
