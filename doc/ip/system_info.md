# System information

This is a special IP block that is automatically populated with useful information about the system.
These values are useful to know what the parameters were when a bitstream was generate.

| Offset | Register |
|--------|----------|
| 0x0000 | First 4 bytes of git commit hash from which this bistream was generated.
| 0x0004 | Second 4 bytes of git commit hash.
| 0x0008 | Git dirty status (1 if dirty or 0 if clean).
| 0x000C | System clock frequency in Hz.
| 0x0010 | GPIO info of which least significant byte is the number of instances.
| 0x0014 | UART info of which least significant byte is the number of instances.
| 0x0018 | I2C info of which least significant byte is the number of instances.
| 0x001C | SPI info of which least significant byte is the number of instances.
