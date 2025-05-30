# GPIO

The General Purpose Input and Output (GPIO) sub-system provides a generic low-speed means of interacting with components outside of the FPGA.
Multiple GPIO instances are provided in an array within the GPIO sub-system.
The first GPIO instance allows Ibex to sense the state of the on-board buttons/switches and to drive the on-board LEDs.
Further GPIO instances are provided for the GPIO pins of the various on-board expansion headers.
Pin-Change INTerrupt (PCINT) logic allows interrupts to be generated from external stimulus.

Note that the PCINT capture logic is not asynchronous.
A pin must change for at least one system-clock cycle (default 40 MHz) to be detected.

## Registers

Each instance has its own self-contained set of registers to aid compartmentalisation.

Most, the registers within most instances share an instance-specific mapping between register bits and external pins.
The common case is where the same pins are used for both input or output, and for register bits to each represent pins.
The exceptions are the on-board peripherals GPIO instance and the control and status register, both of which are covered later on.

### Output & Output Enable Registers

Pins can be driven as outputs by setting matching bits in the output enable register and writing the desired value to the output register.
When writing, it only writes the bits for which the output is set to enable.
When the output enable is set to zero it instead acts as an input pin.

The input and output registers have the same bit mapping for all but the on-board peripherals GPIO instance.

### Input & Debounced Input Registers

Each GPIO instance provides debounced input values as well as raw (pre-debouncing) values in separate registers.
Debouncing is useful when using mechanical switches to avoid counting a single button press multiple times.
For more information on why this is, see [the Wikipedia page on contact bounce](https://en.wikipedia.org/wiki/Switch#Contact_bounce).
Debouncing is performed by checking that any change on an input pin remains stable throughout at least one cycle of a shared hardware timer before the change is propagated to the register.
The alternative 'raw' input register is provided for situations where more precise timing is required.

### PCINT Mask Register

The PCINT mask register allows a custom subset of pins to be used for PCINT generation.
Set a bit to enable monitoring and possible PCINT generation according to the state of the control and status register.
Clear a bit to ignore that pin.

The input and PCINT mask registers have the same bit mapping for all GPIO instances.

### Control & Status Register

The control and status register is currently only used for PCINT functionality.
It is the only register where each bit is not mapped to a physical pin.

| Bit offset | Description |
|------------|-------------|
| 19         | PCINT debounced input select: 0=raw-input, 1=debounced-input. |
| 18-17      | PCINT mode: 0=any-edge, 1=rising-edge, 2=falling-edge, 3=low-level. |
| 16         | PCINT instance-wide enable. Does not affect the status bit. |
| 0          | PCINT status. Write `1` to clear. |

## Memory Map

The addresses of each register of each GPIO instance is given in the following table.

| Offset | Register                 |
|--------|--------------------------|
| 0x0000 | On-board output          |
| 0x0004 | On-board input           |
| 0x0008 | On-board debounced input |
| 0x000C | On-board output enable (currently not used) |
| 0x0010 | On-board PCINT mask      |
| 0x0014 | On-board control & status|
| 0x0040 | R-Pi output              |
| 0x0044 | R-Pi input               |
| 0x0048 | R-pi debounced input     |
| 0x004C | R-pi output enable       |
| 0x0050 | R-Pi PCINT mask          |
| 0x0054 | R-Pi Control & status    |
| 0x0080 | Arduino output           |
| 0x0084 | Arduino input            |
| 0x0088 | Arduino debounced input  |
| 0x008C | Arduino output enable    |
| 0x0090 | Arduino PCINT mask       |
| 0x0094 | Arduino Control & status |
| 0x00C0 | PMOD0 output             |
| 0x00C4 | PMOD0 input              |
| 0x00C8 | PMOD0 debounced input    |
| 0x00CC | PMOD0 output enable      |
| 0x00D0 | PMOD0 PCINT mask         |
| 0x00D4 | PMOD0 Control & status   |
| 0x0100 | PMOD1 output             |
| 0x0104 | PMOD1 input              |
| 0x0108 | PMOD1 debounced input    |
| 0x010C | PMOD1 output enable      |
| 0x0110 | PMOD1 PCINT mask         |
| 0x0114 | PMOD1 Control & status   |
| 0x0140 | PMODC output             |
| 0x0144 | PMODC input              |
| 0x0148 | PMODC debounced input    |
| 0x014C | PMODC output enable      |
| 0x0150 | PMODC PCINT mask         |
| 0x0154 | PMODC Control & status   |

## Instances

There are several GPIO instances, each with their own mapping of external pins to register bits.

### On-board Peripherals

Unlike most GPIO instances here, the on-board peripherals instance uses a different pin mapping for input compared to output.
It interfaces with un-removable user switches, buttons, and LEDs, for each of which only one of input or output makes sense.

The output register drives the user LEDs.

| Bit offset | Description |
|------------|-------------|
| 7-0        | LEDs        |

In this case writing a one will turn an LED on and a zero will turn the LED off.

Note: the output enable register does nothing in this instance, as the outputs are not shared with inputs.

The input registers sense the state of the various user switches/buttons/joystick available on the board, and the microSD card detection line.

| Bit offset | Description |
|------------|-------------|
| 16         | MicroSD card detection (0: present, 1: absent) |
| 15-13      | Software select switches (1, 2, 3) |
| 12-8       | Joystick (left, down, up, right, press) |
| 7-0        | DIP switches |

It is recommended to use the debounced input for these mechanical switches.

### Raspberry Pi HAT

The Raspberry Pi HAT header has 28 pins that can act as GPIO.
Some can be remapped to other IP blocks (see Pinmux documentation).

| Bit offset | Description  |
|------------|--------------|
| 27-0       | GPIO 27 to 0 |

Note: Before using the Raspberry Pi HAT header's GPIO you should use the pinmux to configure them as input or output.

### Arduino Shield

The Arduino Shield header has 14 pins that can act as GPIO.

| Bit offset | Description  |
|------------|--------------|
| 13-0       | GPIO 13 to 0 |

### Pmod

The Pmod header is split up as Pmod 0, C and 1.
Pmod 0 and 1 have 8 GPIO outputs each while C has 6.

| Bit offset | Description |
|------------|-------------|
| 7-6        | Accessible for Pmod 0 and 1 only |
| 5-0        | Accessible for Pmod 0, 1 and C |
