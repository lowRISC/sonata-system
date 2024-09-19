# Sonata Hardware Reference

Sonata consists of a many layers, even just in the hardware.

At its foundation is the Sonata Board.
A Printed Circuit Board (PCB) with many components and expansion ports.
These are fixed and unchanging, except for possible slight changes between revisions.

The central component of the Sonata board is the Field-Programmable Gate Array (FPGA) chip.
Such devices can be configured (using a "bitstream") to model a digital logic design of your choosing.
In this project, we load the Sonata Core onto the FPGA.
FPGAs can be configured and reconfigured any number of times, so the Sonata core can be continuously developed and loaded onto existing boards.

The following sections provide reference information for the [Sonata Core](../dev/ref-manual.md) and the [Sonata Board](../architecture/board.md).
