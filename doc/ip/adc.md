# Analogue to digital converter (ADC)

The FPGA provides an analogue to digital converter, and this hardware IP block provides a memory mapped way of interacting with that.
Please see [the specification for XADC][XADC] for a more detailed description of how it works.

## Registers

| Offset    | Register |
|-----------|----------|
| 0x00-0x3F | Status   |
| 0x40-0x7F | Control  |

For a more detailed description of all of these registers please see [Chapter 3 of the XADC specification][XADC].

[XADC]: https://docs.xilinx.com/r/en-US/ug480_7Series_XADC
