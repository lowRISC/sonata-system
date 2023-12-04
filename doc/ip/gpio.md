# GPIO

General purpose input and output is used for Ibex to interact with the buttons and switches on the board.
It is also used to drive the LEDs on the board.
There are also the GPIO pins of the various headers.

For the input this module provides a raw value as well as a debounced value.
Debouncing is useful to avoid counting a single button press multiple times.
For more information on contact bounce, see [this Wikipedia page](https://en.wikipedia.org/wiki/Switch#Contact_bounce).

| Offset | Register |
|--------|----------|
| 0x00   | Output   |
| 0x04   | Input    |
| 0x08   | Debounced input |
| 0x0C   | Debounce threshold |
| 0x10   | Raspberry pi header |
| 0x14   | Raspberry pi mask |
| 0x18   | Arduino shield header |
| 0x1C   | Arduino shield mask |

## Output

The output register displays the specified value onto the boards output.

| Bit offset | Description |
|------------|-------------|
| 7-0        | LEDs        |

In this case writing a one will turn an LED on and a zero will turn the LED off.

## Input

Both input registers have the same bit mapping.
The only difference between the registers is that the latter has debounced signals.

| Bit offset | Description |
|------------|-------------|
| 13-9       | Joystick (left, down, up, right, press) |
| 8          | Button |
| 7-0        | DIP switches |

The input registers are used to interact with the joystick, the button and the DIP switches that are available on the Sonata board.

## Debounce threshold

This register indicates how many clock cycles an input needs to be stable before it shows up on the output.
The same threshold applies to all of the inputs.

| Bit offset | Description |
|------------|-------------|
| 31-0       | Threshold   |

## Raspberry Pi header

The Raspberry Pi header has 26 GPIO pins, this register can read from and write to these pins.
When writing, it only writes the bits for which the write mask has a one.
The header and write mask registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|  24        | GPIO 26     |
| ...        | ...         |
|   1        | GPIO  3     |
|   0        | GPIO  2     |

## Arduino shield header

Arduino shield headers have 13 IO pins, this register can read from and write to these pins.
When writing, it only writes the bits for which the write mask has a one.
The header and the write mask registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
|  13        | IO 13       |
| ...        | ...         |
|   1        | IO  1       |
|   0        | IO  0       |
