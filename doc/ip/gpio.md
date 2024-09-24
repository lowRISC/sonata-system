# GPIO

General purpose input and output is used for Ibex to interact with the buttons and switches on the board.
It is also used to drive the LEDs on the board.
There are also the GPIO pins of the various headers.

For the input this module provides a raw value as well as a debounced value.
Debouncing is useful to avoid counting a single button press multiple times.
For more information on contact bounce, see [this Wikipedia page](https://en.wikipedia.org/wiki/Switch#Contact_bounce).

| Offset | Register                |
|--------|-------------------------|
| 0x0000 | Output                  |
| 0x0004 | Input                   |
| 0x0008 | Debounced input         |
| 0x000C | Output enable           |
| 0x0010 | R-Pi output             |
| 0x0014 | R-Pi input              |
| 0x0018 | R-pi debounced input    |
| 0x001C | R-pi output enable      |
| 0x0020 | Arduino output          |
| 0x0024 | Arduino input           |
| 0x0028 | Arduino debounced input |
| 0x002C | Arduino output enable   |
| 0x0030 | PMOD output             |
| 0x0034 | PMOD input              |
| 0x0038 | PMOD debounced input    |
| 0x003C | PMOD output enable      |

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
| 16-14      | Software select switches (1, 2, 3) |
| 13         | mikroBUS interrupt |
| 12-5       | DIP switches |
| 4-0        | Joystick (left, down, up, right, press) |

The input registers are used to interact with the joystick, the button and the DIP switches that are available on the Sonata board.

## Raspberry Pi HAT

The Raspberry Pi HAT header has 27 pins that can act as GPIO.
Some can be remapped to other IP blocks (see Pinmux).
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
| 27-0       | g27 - g0    |

When the output enable is set to zero it instead acts as an input pin.

## Arduino Shield

The Arduino Shield header has 17 pins that can act as GPIO.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description  |
|------------|--------------|
| 17-0       | GPIO 17 to 0 |

When the output enable is set to zero it instead acts as an input pin.

## PMOD

The two PMOD headers has 16 GPIO pins.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|   15-8     | PMOD 1      |
|   7-0      | PMOD 0      |

When the output enable is set to zero it instead acts as an input pin.
