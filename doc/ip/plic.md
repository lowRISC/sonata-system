# Platform-level interrupt controller (PLIC)

The PLIC is a RISC-V interrupt controller and specifically we are using [OpenTitan's interrupt controller](https://opentitan.org/book/hw/top_earlgrey/ip_autogen/rv_plic/index.html).
Please find more details in [the official specification](https://github.com/riscv/riscv-plic-spec/releases/download/1.0.0/riscv-plic-1.0.0.pdf).
Part of these details are is the memory map, which shows all registers from the `base` until `base + 0x3FFFFFC`.
Not all of these registers are mapped, most importantly we only have one core, so only one context (context 0).

The following table shows the current list of interrupts that are fed through the PLIC.
For more detailed descriptions of what each interrupt means, please refer to the documentation of each individual hardware IP block.
More interrupts may be added to this table in the future.

| Number | Block     | Interrupt description |
|--------|-----------|-----------------------|
|  0     | None      | Tied to zero
|  1,  9 | UART 0, 1 | Transmit watermark
|  2, 10 | UART 0, 1 | Receive watermark
|  3, 11 | UART 0, 1 | Transmit empty
|  4, 12 | UART 0, 1 | Receive overflow
|  5, 13 | UART 0, 1 | Receive frame error
|  6, 14 | UART 0, 1 | Receive break error
|  7, 15 | UART 0, 1 | Receive timeout
|  8, 16 | UART 0, 1 | Receive parity error
| 17, 32 | I2C 0, 1  | Format FIFO threshold
| 18, 33 | I2C 0, 1  | Receive FIFO threshold
| 19, 34 | I2C 0, 1  | Acquire FIFO threshold
| 20, 35 | I2C 0, 1  | Receive FIFO overflow
| 21, 36 | I2C 0, 1  | Received NACK
| 22, 37 | I2C 0, 1  | SCL interference
| 23, 38 | I2C 0, 1  | SDA interference
| 24, 39 | I2C 0, 1  | Stretch timeout
| 25, 40 | I2C 0, 1  | SDA unstable
| 26, 41 | I2C 0, 1  | Command complete
| 27, 42 | I2C 0, 1  | Transmit stretch
| 28, 43 | I2C 0, 1  | Transmit threshold
| 29, 44 | I2C 0, 1  | Acquire FIFO full
| 30, 45 | I2C 0, 1  | Unexpected stop
| 31, 46 | I2C 0, 1  | Host timeout
| 47     | Ethernet  | Interrupt from external SPI ethernet chip (KSZ8851SNLI-TR). Check the interrupt status register for details.

