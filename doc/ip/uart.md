# UART

| Offset | Register |
|--------|----------|
| 0x00   | Receive |
| 0x04   | Transmit |
| 0x08   | Status |

Receive data:

| Bit offset | Description |
|------------|-------------|
| 7-0        | Received byte |

Transmit data:

| Bit offset | Description |
|------------|-------------|
| 7-0        | Byte to write to UART |

Status:

| Bit offset | Description |
|------------|-------------|
| 1          | Transmit FIFO full |
| 0          | Receive FIFO empty |
