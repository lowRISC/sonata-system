# SPI host

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
