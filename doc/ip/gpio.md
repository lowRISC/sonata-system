# GPIO

General purpose input and output is used for Ibex to interact with the buttons and switches on the board.
It is also used to drive the LEDs on the board.
There are also the GPIO pins of the various headers.

For the input this module provides a raw value as well as a debounced value.
Debouncing is useful to avoid counting a single button press multiple times.
For more information on contact bounce, see [this Wikipedia page](https://en.wikipedia.org/wiki/Switch#Contact_bounce).

| Offset | Register |
|--------|----------|
| 0x0000 | Output   |
| 0x0004 | Input    |
| 0x0008 | Debounced input |
| 0x000C | Debounce threshold |
| 0x6000 | R-Pi output |
| 0x6004 | R-Pi input |
| 0x6008 | R-pi debounced input |
| 0x600C | R-pi debounce threshold |
| 0x7000 | Arduino output |
| 0x7004 | Arduino input |
| 0x7008 | Arduino debounced input |
| 0x700C | Arduino debounce threshold |
| 0x8000 | PMOD output |
| 0x8004 | PMOD input |
| 0x8008 | PMOD debounced input |
| 0x800C | PMOD debounce threshold |

## Output

The output register displays the specified value onto the boards output.

| Bit offset | Description |
|------------|-------------|
| 22         | mikroBUS reset |
| 21         | mikroBUS SPI chip select |
| 20         | Arduino SPI chip select |
| 19         | R-Pi SPI1 chip select 0 |
| 18         | R-Pi SPI1 chip select 1 |
| 17         | R-Pi SPI1 chip select 2 |
| 16         | R-Pi SPI0 chip select 0 |
| 15         | R-Pi SPI0 chip select 1 |
| 14         | Ethernet reset |
| 13         | Ethernet SPI chip select |
| 12         | Flash SPI chip select |
| 11-4       | LEDs        |
| 3          | LCD backlight |
| 2          | LCD DC      |
| 1          | LCD reset   |
| 0          | LCD chip select |

In this case writing a one will turn an LED on and a zero will turn the LED off.

## Input

Both input registers have the same bit mapping.
The only difference between the registers is that the latter has debounced signals.

| Bit offset | Description |
|------------|-------------|
| 13         | mikroBUS interrupt |
| 12-5       | DIP switches |
| 4-0        | Joystick (left, down, up, right, press) |

The input registers are used to interact with the joystick, the button and the DIP switches that are available on the Sonata board.

## Debounce threshold

**Not implemented yet.**

This register indicates how many clock cycles an input needs to be stable before it shows up on the output.
The same threshold applies to all of the inputs.

| Bit offset | Description |
|------------|-------------|
| 31-0       | Threshold   |

## Raspberry Pi HAT

The Raspberry Pi HAT header has 11 GPIO pins.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|   10       | GPIO  27    |
|   9        | GPIO  26    |
|   8        | GPIO  25    |
|   7        | GPIO  24    |
|   6        | GPIO  23    |
|   5        | GPIO  22    |
|   4        | GPIO  13    |
|   3        | GPIO  12    |
|   2        | GPIO  6     |
|   1        | GPIO  5     |
|   0        | GPIO  4     |

For the output register it is teh same order as above:
| Bit offset | Description |
|------------|-------------|
|   26-16    | Output enable |
|   10-0     | Output value |

When the output enable is set to zero it instead acts as an input pin.

## Arduino Shield

The Arduino Shield header has 10 GPIO pins.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|   9        | GPIO  9     |
|   8        | GPIO  8     |
|   7        | GPIO  7     |
|   6        | GPIO  6     |
|   5        | GPIO  5     |
|   4        | GPIO  4     |
|   3        | GPIO  3     |
|   2        | GPIO  2     |
|   1        | GPIO  1     |
|   0        | GPIO  0     |

For the output register it is teh same order as above:
| Bit offset | Description |
|------------|-------------|
|   25-16    | Output enable |
|   9-0      | Output value |

When the output enable is set to zero it instead acts as an input pin.

## PMOD

The two PMOD headers has 16 GPIO pins.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|   15-8     | PMOD 1      |
|   7-0      | PMOD 0      |

For the output register it is teh same order as above:
| Bit offset | Description |
|------------|-------------|
|   31-16    | Output enable |
|   15-0     | Output value |

When the output enable is set to zero it instead acts as an input pin.
