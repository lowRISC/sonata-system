# GPIO

General purpose input and output is used for Ibex to interact with the buttons and switches on the board.
It is also used to drive the LEDs on the board.
There are also the GPIO pins of the various headers.

For the input this module provides a raw value as well as a debounced value.
Debouncing is useful to avoid counting a single button press multiple times.
For more information on contact bounce, see [this Wikipedia page](https://en.wikipedia.org/wiki/Switch#Contact_bounce).

| Offset | Register                 |
|--------|--------------------------|
| 0x0000 | Output                   |
| 0x0004 | Input                    |
| 0x0008 | Debounced input          |
| 0x000C | Output enable (currently not used) |
| 0x0040 | R-Pi output              |
| 0x0044 | R-Pi input               |
| 0x0048 | R-pi debounced input     |
| 0x004C | R-pi output enable       |
| 0x0080 | Arduino output           |
| 0x0084 | Arduino input            |
| 0x0088 | Arduino debounced input  |
| 0x008C | Arduino output enable    |
| 0x00C0 | PMOD0 output             |
| 0x00C4 | PMOD0 input              |
| 0x00C8 | PMOD0 debounced input    |
| 0x00CC | PMOD0 output enable      |
| 0x0100 | PMOD1 output             |
| 0x0104 | PMOD1 input              |
| 0x0108 | PMOD1 debounced input    |
| 0x010C | PMOD1 output enable      |
| 0x0140 | PMODC output             |
| 0x0144 | PMODC input              |
| 0x0148 | PMODC debounced input    |
| 0x014C | PMODC output enable      |

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
| 16         | MicroSD card detection (0: present, 1: absent) |
| 15-13      | Software select switches (1, 2, 3) |
| 12-8       | Joystick (left, down, up, right, press) |
| 7-0        | DIP switches |

The input registers are used to interact with the joystick, the button and the DIP switches that are available on the Sonata board.

## Raspberry Pi HAT

The Raspberry Pi HAT header has 28 pins that can act as GPIO.
Some can be remapped to other IP blocks (see Pinmux).
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description  |
|------------|--------------|
| 27-0       | GPIO 27 to 0 |

When the output enable is set to zero it instead acts as an input pin.

## Arduino Shield

The Arduino Shield header has 14 pins that can act as GPIO.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description  |
|------------|--------------|
| 13-0       | GPIO 13 to 0 |

When the output enable is set to zero it instead acts as an input pin.

## Pmod

The Pmod header is split up as Pmod 0, C and 1.
Pmod 0 and 1 have 8 GPIO outputs each while C has 6.
When writing, it only writes the bits for which the output is set to enable.
The input and output registers have the same bit mapping.

| Bit offset | Description |
|------------|-------------|
| 7-6        | Accessible for Pmod 0 and 1 only |
| 5-0        | Accessible for Pmod 0, 1 and C |

When the output enable is set to zero it instead acts as an input pin.
