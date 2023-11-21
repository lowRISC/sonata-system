# Sonata architecture specification

The Sonata architecture comprises a number of components:
- Physical board architecture:
  This is the board that hosts the FPGA (field programmable gate array), all other components, headers and interfaces.
- FPGA configuration architecture:
  It defines everything we are using a hardware description language for.
  This includes hardware IP blocks and bus architecture.
- Software architecture:
  This includes toolchain, operating system and applications.

Before we go into the architecture of the system, it is good to understand the use cases that we are envisioning, so that we can derive our architectural requirements from that.

## Use cases and requirements

The Sonata is meant to be used by academics and industry users who are interested in experimenting with CHERIoT in embedded and IoT applications.
This is the main reason why we are building a custom FPGA board so that we can make the platform easily *usable*.
We also need the board to be as *affordable* as possible while still being performant and usable.

This ease of use comes in handy for classroom and demonstration use cases, for which we think this board will come in quite handy.
In the classroom, it is also pertinent that we have a *debuggable* system.

Because we are focussing on embedded and IoT applications, we need to ensure connectivity.
This includes being *connectable* to standard peripherals as well as being *extendable* with functionality required for niche use-cases.

The other major benefit of creating a custom board is that we can highlight CHERIoT specific features.
We envision users of this board to want to show off and experiment with the new CHERIoT technology.
If they are using the board as a demonstrator they will want to show off the prowess of CHERI, and if they are experimenting with the board they will want *CHERI to be visible*.

Like most physical boards, it is nice for it to be as interactive as possible.
This means being able to accept user input and respond to them visibly and in an *interactable* way.

Beyond the physical Sonata board, we expect the soft design that is used to configure the FPGA to also be *integrable* into a bigger design.
For example, we envision it being connected to OpenTitan Earl Grey through a bridge interface.

In short, these are our general requirements:
- Usable
- Affordable
- Debuggable
- Connectable
- Extendable
- CHERI visible
- Interactive
- Integrable

## Detailed specifications

Please have a look at the following documents for more detailed architecture specifications:
- [Physical board](board.md)
- [FPGA configuration](fpga.md)
- [Software](software.md)

## Conclusion

The Sonata system focuses on being:
- Usable
- Connectable
- Extendable
- Debuggable
- CHERI visible
- Interactable
- Affordable
- Integratable

The goal of the Sunburst project is to get CHERIoT hardware into the hands of engineers and Sonata is a crucial step in that process.
