# Analogue to digital converter (ADC)

The FPGA provides an analogue to digital converter called XADC, and this hardware IP block provides a memory mapped way of interacting with that.
Please see [the specification for XADC][XADC] for a more detailed description of how it works.

## Analogue Input Channels

The six Arduino shield analogue input pins (A0-A5) are connected to six of the XADC Vaux channels.

| Pin | Channel  |
|-----|----------|
| A0  | Vaux[4]  |
| A1  | Vaux[12] |
| A2  | Vaux[5]  |
| A3  | Vaux[13] |
| A4  | Vaux[6]  |
| A5  | Vaux[14] |

The input range at the shield pin is 0-5V.
Conversion from a single-ended 0-5V signal at the shield pin to a 0-1V differential signal at the XADC inputs is handled by on-board buffering and balancing circuit.
Note that impedance drops above 3.3V due to diode clamps on a resistor-gated branch that also connects each pin to a digital FPGA input (currently unused).

## Sensors

A set of on-chip sensors can also be read by the XADC.
The most interesting of these is a temperature sensor.
The rest are various supply/reference voltages.

## Registers

The XADC is accessed via its Dynamic Reconfiguration Port (DRP), which give access to various 16-bit control and status registers.
These registers have been memory mapped at a 32-bit intervals to simplify accesses.

| DRP Address       | Register |
|-------------------|----------|
| 0x00 0x01 .. 0x3F | Status   |
| 0x40 0x41 .. 0x7F | Control  |

| Memory-Mapped Offset | Register |
|----------------------|----------|
| 0x000 0x004 .. 0x0FC | Status   |
| 0x100 0x104 .. 0x1FC | Control  |

By default, the XADC will continuously sample all six routed analogue inputs and all on-chip sensors at a low sample rate, with conversion results ready to be read by the host from the status registers.
This default configuration is achieved by specifying initial values for the configuration registers in the RTL.
The sample rate or other configuration can be changed by writing to the control registers.

For a more detailed description of all of these registers please see [Chapter 3 of the XADC specification][XADC].

[XADC]: https://docs.xilinx.com/r/en-US/ug480_7Series_XADC
