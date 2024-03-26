# Serial peripheral interface (SPI) host

The SPI in Sonata only has the capability to be a host.
This is a simple hardware IP block that can transmit and receive bytes over SPI.

In Sonata, there are multiple uses for SPI:
- LCD screen
- Ethernet block
- Raspberry Pi hat
- Arduino shield
- mikroBUS

By default Sonata contains 3 SPI blocks.
One of which should be used for the LCD, the other two can be configured to be connected to the target using the pin multiplexer.

The offset for each of the blocks is shown below, with each additional block having a `0x1000` offset from the previous.

| Offset | Register |
|--------|----------|
| 0x00   | Transmit |
| 0x04   | Receive  |
| 0x08   | Status   |
| 0x0C   | Config   |

## Transmit

The transmit register is used to write data to the SPI one byte at a time.

| Bit offset | Description |
|------------|-------------|
| 7-0        | Write data  |

## Receive

The receive register is used to read data to the SPI one byte at a time.

| Bit offset | Description |
|------------|-------------|
| 7-0        | Read data   |

## Status

The status register is an insight into the internal state of the SPI host.

| Bit offset  | Description         |
|-------------|---------------------|
| 4           | Receive error       |
| 3           | Receive  FIFO empty |
| 2           | Receive  FIFO full  |
| 1           | Transmit FIFO empty |
| 0           | Transmit FIFO full  |

## Config

This register provides multiple configuration options:

| Bit offset | Description   |
|------------|---------------|
| 31         | CPOL          |
| 30         | CPHA          |
| 29         | Reset receive |
| 28         | Reset transmit |
| 15-0       | Clock divider |

CPOL indicates the clock polarity.
If set to one, the clock is high when idle.

CPHA indicates the phase of the clock.
When this is set to zero, data changes on the trailing edge.
When this is set to one, data changes on the leading edge.

Reset receive clears the receive FIFO and any potential receive error, while reset transmit clears the transmit FIFO.

The clock divider indicates what the SPI clock should be like with respect to the core clock.
