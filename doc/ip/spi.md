# Serial peripheral interface (SPI) host

The SPI in Sonata only has the capability to be a host.
This is a simple hardware IP block that can transmit and receive bytes over SPI.

In Sonata, there are multiple uses for SPI:
- LCD screen
- Ethernet
- Flash
- MicroSD card
- Raspberry Pi HAT
- Arduino shield
- mikroBUS click

The offset for each of the blocks is shown below, with each additional block having a 0x1000 offset from the previous.
The offsets in the table below are from the SPI base address, which is 0x8030_0000.
The table also shows the connections of the chip select (CS) wires.
CS1 for LCD and Ethernet and CS2 for LCD are *not* used as chip selects but rather for block specific functions like reset and data/control signals.

| Name         | Offset | Blocks                    | CS0          | CS1          | CS2          | CS3      |
|--------------|--------|---------------------------|--------------|--------------|--------------|----------|
| LCD Screen   | 0x0000 | *Only* LCD                | CS           | Data/control | Reset        |          |
| Ethernet MAC | 0x1000 | *Only* Ethernet           | CS           | Reset        |              |          |
| SPI0         | 0x2000 | Flash or MicroSD          | Flash        | MicroSD      |              |          |
| SPI1         | 0x3000 | HAT SPI0, Shield, Pmod0   | HAT or Pmod0 | HAT or Pmod0 | Pmod0        | Shield   |
| SPI2         | 0x4000 | HAT SPI1, mikroBUS, Pmod1 | HAT or Pmod1 | HAT or Pmod1 | HAT or Pmod1 | mikroBUS |

Please refer to the [pin multiplexer](pinmux/README.md) and [pin mappings](pinmux/pin-mappings.md) on how to connect SPI 0, 1 and 2.

## Overview

Each SPI block has two 8-entry FIFOs - one for transmit and one for receive.
To begin an SPI transaction write to the [`START`](#start) register.
Bytes do not need to be immediately available in the transmit FIFO nor space available in the receive FIFO to begin the transaction.
The SPI block will only run the clock when its able to proceed.

**Note Interrupts are not yet implemented**


## Register Table

| Name                              | Offset   |   Length | Description                                                       |
|:----------------------------------|:---------|---------:|:------------------------------------------------------------------|
| spi.[`INTR_STATE`](#intr_state)   | 0x0      |        4 | Interrupt State Register                                          |
| spi.[`INTR_ENABLE`](#intr_enable) | 0x4      |        4 | Interrupt Enable Register                                         |
| spi.[`INTR_TEST`](#intr_test)     | 0x8      |        4 | Interrupt Test Register                                           |
| spi.[`CFG`](#cfg)                 | 0xc      |        4 | Configuration register.                                           |
| spi.[`CONTROL`](#control)         | 0x10     |        4 | Controls the operation of the SPI block.                          |
| spi.[`STATUS`](#status)           | 0x14     |        4 | Status information about the SPI block.                           |
| spi.[`START`](#start)             | 0x18     |        4 | When written begins an SPI operation.                             |
| spi.[`RX_FIFO`](#rx_fifo)         | 0x1c     |        4 | Data from the receive FIFO.                                       |
| spi.[`TX_FIFO`](#tx_fifo)         | 0x20     |        4 | Bytes written here are pushed to the transmit FIFO.               |
| spi.[`INFO`](#info)               | 0x24     |        4 | Returns information on the SPI controller.                        |
| spi.[`CS`](#cs)                   | 0x28     |        4 | Specifies which peripherals are selected for SPI operations.      |

## INTR_STATE
Interrupt State Register
- Offset: `0x0`
- Reset default: `0x0`
- Reset mask: `0x1f`

### Fields

```wavejson_reg
[{"name": "rx_full", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "rx_watermark", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "tx_empty", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "tx_watermark", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "complete", "bits": 1, "attr": ["rw1c"], "rotate": -90}, {"bits": 27}]
```

|  Bits  |  Type  |  Reset  | Name         | Description                                                    |
|:------:|:------:|:-------:|:-------------|:---------------------------------------------------------------|
|  31:5  |        |         |              | Reserved                                                       |
|   4    |  rw1c  |   0x0   | complete     | On-going SPI operation has completed and the block is now idle |
|   3    |   ro   |   0x0   | tx_watermark | Transmit FIFO level is at or below watermark                   |
|   2    |   ro   |   0x0   | tx_empty     | Transmit FIFO is empty                                         |
|   1    |   ro   |   0x0   | rx_watermark | Receive FIFO level is at or above watermark                    |
|   0    |   ro   |   0x0   | rx_full      | Receive FIFO is full                                           |

## INTR_ENABLE
Interrupt Enable Register
- Offset: `0x4`
- Reset default: `0x0`
- Reset mask: `0x1f`

### Fields

```wavejson_reg
[{"name": "rx_full", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "rx_watermark", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "tx_empty", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "tx_watermark", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "complete", "bits": 1, "attr": ["rw"], "rotate": -90}, {"bits": 27}]
```

|  Bits  |  Type  |  Reset  | Name         | Description                                                            |
|:------:|:------:|:-------:|:-------------|:-----------------------------------------------------------------------|
|  31:5  |        |         |              | Reserved                                                               |
|   4    |   rw   |   0x0   | complete     | Enable interrupt when [`INTR_STATE.complete`](#intr_state) is set.     |
|   3    |   rw   |   0x0   | tx_watermark | Enable interrupt when [`INTR_STATE.tx_watermark`](#intr_state) is set. |
|   2    |   rw   |   0x0   | tx_empty     | Enable interrupt when [`INTR_STATE.tx_empty`](#intr_state) is set.     |
|   1    |   rw   |   0x0   | rx_watermark | Enable interrupt when [`INTR_STATE.rx_watermark`](#intr_state) is set. |
|   0    |   rw   |   0x0   | rx_full      | Enable interrupt when [`INTR_STATE.rx_full`](#intr_state) is set.      |

## INTR_TEST
Interrupt Test Register
- Offset: `0x8`
- Reset default: `0x0`
- Reset mask: `0x1f`

### Fields

```wavejson_reg
[{"name": "rx_full", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "rx_watermark", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "tx_empty", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "tx_watermark", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "complete", "bits": 1, "attr": ["wo"], "rotate": -90}, {"bits": 27}]
```

|  Bits  |  Type  |  Reset  | Name         | Description                                                     |
|:------:|:------:|:-------:|:-------------|:----------------------------------------------------------------|
|  31:5  |        |         |              | Reserved                                                        |
|   4    |   wo   |   0x0   | complete     | Write 1 to force [`INTR_STATE.complete`](#intr_state) to 1.     |
|   3    |   wo   |   0x0   | tx_watermark | Write 1 to force [`INTR_STATE.tx_watermark`](#intr_state) to 1. |
|   2    |   wo   |   0x0   | tx_empty     | Write 1 to force [`INTR_STATE.tx_empty`](#intr_state) to 1.     |
|   1    |   wo   |   0x0   | rx_watermark | Write 1 to force [`INTR_STATE.rx_watermark`](#intr_state) to 1. |
|   0    |   wo   |   0x0   | rx_full      | Write 1 to force [`INTR_STATE.rx_full`](#intr_state) to 1.      |

## CFG
Configuration register. Controls how the SPI block transmits
   and receives data. This register can only be modified
   whilst the SPI block is idle.

- Offset: `0xc`
- Reset default: `0x20000000`
- Reset mask: `0xe000ffff`

### Fields

```wavejson_reg
[{"name": "HALF_CLK_PERIOD", "bits": 16, "attr": ["rw"], "rotate": 0}, {"bits": 13}, {"name": "MSB_FIRST", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CPHA", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "CPOL", "bits": 1, "attr": ["rw"], "rotate": -90}]
```

|  Bits  |  Type  |  Reset  | Name                                     |
|:------:|:------:|:-------:|:-----------------------------------------|
|   31   |   rw   |   0x0   | [CPOL](#cfg--cpol)                       |
|   30   |   rw   |   0x0   | [CPHA](#cfg--cpha)                       |
|   29   |   rw   |   0x1   | [MSB_FIRST](#cfg--msb_first)             |
| 28:16  |        |         | Reserved                                 |
|  15:0  |   rw   |   0x0   | [HALF_CLK_PERIOD](#cfg--half_clk_period) |

### CFG . CPOL
The polarity of the spi_clk signal. When CPOL is 0 clock is
   low when idle and the leading edge is positive. When CPOL
   is 1 clock is high when idle and the leading edge is
   negative

### CFG . CPHA
The phase of the spi_clk signal. When CPHA is 0 data is
   sampled on the leading edge and changes on the trailing
   edge. The first data bit is immediately available before
   the first leading edge of the clock when transmission
   begins. When CPHA is 1 data is sampled on the trailing edge
   and change on the leading edge.

### CFG . MSB_FIRST
When set the most significant bit (MSB) is the first bit
   sent and received with each byte

### CFG . HALF_CLK_PERIOD
The length of a half period (i.e. positive edge to negative
   edge) of the SPI clock, measured in system clock cycles
   reduced by 1. At the standard Sonata 50 MHz system clock a
   value of 0 gives a 25 MHz SPI clock, a value of 1 gives a
   12.5 MHz SPI clock, a value of 2 gives a 8.33 MHz SPI clock
   and so on.

## CONTROL
Controls the operation of the SPI block. This register can
   only be modified whilst the SPI block is idle.
- Offset: `0x10`
- Reset default: `0x0`
- Reset mask: `0xfff`

### Fields

```wavejson_reg
[{"name": "TX_CLEAR", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "RX_CLEAR", "bits": 1, "attr": ["wo"], "rotate": -90}, {"name": "TX_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "RX_ENABLE", "bits": 1, "attr": ["rw"], "rotate": -90}, {"name": "TX_WATERMARK", "bits": 4, "attr": ["rw"], "rotate": -90}, {"name": "RX_WATERMARK", "bits": 4, "attr": ["rw"], "rotate": -90}, {"bits": 20}]
```

|  Bits  |  Type  |  Reset  | Name                                   |
|:------:|:------:|:-------:|:---------------------------------------|
| 31:12  |        |         | Reserved                               |
|  11:8  |   rw   |   0x0   | [RX_WATERMARK](#control--rx_watermark) |
|  7:4   |   rw   |   0x0   | [TX_WATERMARK](#control--tx_watermark) |
|   3    |   rw   |   0x0   | [RX_ENABLE](#control--rx_enable)       |
|   2    |   rw   |   0x0   | [TX_ENABLE](#control--tx_enable)       |
|   1    |   wo   |   0x0   | [RX_CLEAR](#control--rx_clear)         |
|   0    |   wo   |   0x0   | [TX_CLEAR](#control--tx_clear)         |

### CONTROL . RX_WATERMARK
The watermark level for the receive FIFO, depending on the value the interrupt will trigger at different points:
- 0: 1 or more items in the FIFO
- 1: 2 or more items in the FIFO
- 2: 4 or more items in the FIFO
- 3: 8 or more items in the FIFO
- 4: 16 or more items in the FIFO
- 5: 32 or more items in the FIFO
- 6: 56 or more items in the FIFO

### CONTROL . TX_WATERMARK
The watermark level for the transmit FIFO, depending on the value the interrupt will trigger at different points:
- 0: 1 or fewer items in the FIFO
- 1: 2 or fewer items in the FIFO
- 2: 4 or fewer items in the FIFO
- 3: 8 or fewer items in the FIFO
- 4: 16 or fewer items in the FIFO

### CONTROL . RX_ENABLE
When set incoming bits are written to the receive FIFO.
When clear incoming bits are ignored.

### CONTROL . TX_ENABLE
When set bytes from the transmit FIFO are sent.
When clear the state of the outgoing spi_cipo is undefined whilst the SPI clock is running.

### CONTROL . RX_CLEAR
Write 1 to clear the receive FIFO.

### CONTROL . TX_CLEAR
Write 1 to clear the transmit FIFO.

## STATUS
Status information about the SPI block
- Offset: `0x14`
- Reset default: `0x0`
- Reset mask: `0x7ffff`

### Fields

```wavejson_reg
[{"name": "TX_FIFO_LEVEL", "bits": 8, "attr": ["ro"], "rotate": 0}, {"name": "RX_FIFO_LEVEL", "bits": 8, "attr": ["ro"], "rotate": 0}, {"name": "TX_FIFO_FULL", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "RX_FIFO_EMPTY", "bits": 1, "attr": ["ro"], "rotate": -90}, {"name": "IDLE", "bits": 1, "attr": ["ro"], "rotate": -90}, {"bits": 13}]
```

|  Bits  |  Type  |  Reset  | Name          | Description                                                                     |
|:------:|:------:|:-------:|:--------------|:--------------------------------------------------------------------------------|
| 31:19  |        |         |               | Reserved                                                                        |
|   18   |   ro   |    x    | IDLE          | When set the SPI block is idle and can accept a new start command.              |
|   17   |   ro   |    x    | RX_FIFO_EMPTY | When set the receive FIFO is empty and any data read from it will be undefined. |
|   16   |   ro   |    x    | TX_FIFO_FULL  | When set the transmit FIFO is full and any data written to it will be ignored.  |
|  15:8  |   ro   |    x    | RX_FIFO_LEVEL | Number of items in the receive FIFO                                             |
|  7:0   |   ro   |    x    | TX_FIFO_LEVEL | Number of items in the transmit FIFO                                            |

## START
When written begins an SPI operation. Writes are ignored when the SPI block is active.
- Offset: `0x18`
- Reset default: `0x0`
- Reset mask: `0x7ff`

### Fields

```wavejson_reg
[{"name": "BYTE_COUNT", "bits": 11, "attr": ["wo"], "rotate": 0}, {"bits": 21}]
```

|  Bits  |  Type  |  Reset  | Name       | Description                                              |
|:------:|:------:|:-------:|:-----------|:---------------------------------------------------------|
| 31:11  |        |         |            | Reserved                                                 |
|  10:0  |   wo   |   0x0   | BYTE_COUNT | Number of bytes to receive/transmit in the SPI operation |

## RX_FIFO
Data from the receive FIFO. When read the data is popped from the FIFO. If the FIFO is empty data read is undefined.
- Offset: `0x1c`
- Reset default: `0x0`
- Reset mask: `0xff`

### Fields

```wavejson_reg
[{"name": "DATA", "bits": 8, "attr": ["ro"], "rotate": 0}, {"bits": 24}]
```

|  Bits  |  Type  |  Reset  | Name   | Description               |
|:------:|:------:|:-------:|:-------|:--------------------------|
|  31:8  |        |         |        | Reserved                  |
|  7:0   |   ro   |    x    | DATA   | Byte popped from the FIFO |

## TX_FIFO
Bytes written here are pushed to the transmit FIFO. If the FIFO is full writes are ignored.
- Offset: `0x20`
- Reset default: `0x0`
- Reset mask: `0xff`

### Fields

```wavejson_reg
[{"name": "DATA", "bits": 8, "attr": ["wo"], "rotate": 0}, {"bits": 24}]
```

|  Bits  |  Type  |  Reset  | Name   | Description              |
|:------:|:------:|:-------:|:-------|:-------------------------|
|  31:8  |        |         |        | Reserved                 |
|  7:0   |   wo   |   0x0   | DATA   | Byte to push to the FIFO |

## INFO
Returns information on the SPI controller.
- Offset: `0x24`
- Reset default: `0x0`
- Reset mask: `0xff`

### Fields

```wavejson_reg
[{"name": "TX_FIFO_DEPTH", "bits": 8, "attr": ["ro"], "rotate": 0}, {"name": "RX_FIFO_DEPTH", "bits": 8, "attr": ["ro"], "rotate": 0}, {"bits": 16}]
```

|  Bits  |  Type  |  Reset  | Name          | Description                                   |
|:------:|:------:|:-------:|:--------------|:----------------------------------------------|
|  31:16 |        |         |               | Reserved                                      |
|  15:8  |   ro   |   0x0   | RX_FIFO_DEPTH | Maximum number of items in the receive FIFO.  |
|  7:0   |   ro   |   0x0   | TX_FIFO_DEPTH | Maximum number of items in the transmit FIFO. |

## CS
Specifies which peripherals are selected for transmit/receive operations.
An operation may select multiple peripherals simultaneously but this functionality shall be used only for transmit operations.
This register shall be changed only when the SPI controller is idle, not whilst a transmit/receive operation may be in progress.
- Offset: `0x28`
- Reset default: `0xff`
- Reset mask: `0xff`

### Fields

```wavejson_reg
[{"name": "CS", "bits": 4, "attr": ["rw"], "rotate": 0}, {"bits": 28}]
```

|  Bits  |  Type  |  Reset  | Name   | Description              |
|:------:|:------:|:-------:|:-------|:-------------------------|
|  31:5  |        |         |        | Reserved                 |
|  4:0   |   rw   |   0xf   | cs     | If this bit is clear the peripheral is selected for transmit/receive operations. |
