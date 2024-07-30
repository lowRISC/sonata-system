# Developer Guide

The Sonata architecture comprises a number of components:
- Physical board architecture:
  This is the board that hosts the FPGA (field programmable gate array), all other components, headers and interfaces.
- FPGA configuration architecture:
  It defines everything we are using a hardware description language for.
  This includes hardware IP blocks and bus architecture.
- Software architecture:
  This includes toolchain, operating system and applications.

Before we go into the architecture of the system, it is good to understand the use cases that we are envisioning, so that we can derive our architectural requirements from that.

### Use cases and requirements

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

### Toolchain

The toolchain will build on top of the work already done on CHERIoT, which uses a [fork of LLVM](https://github.com/CHERIoT-Platform/llvm-project).
Through Sonata, we are not proposing any changes to the CHERIoT instruction set.
We may need some changes to allow code to be stored in memory that does not have associated tags.

The toolchain for software development is described in the [Software toolchain setup](../dev/toolchain-setup.md) section. If these instructions ever go out of date, you should be able to find the up to date instructions to build the toolchain [from the CI YAML file](https://github.com/CHERIoT-Platform/llvm-project/blob/cheriot/.cirrus.yml).

### Applications

Especially for the usable and CHERI-visible requirements, it is important that we have a set of demonstration applications.
One demonstration application is cycling through each of the CHERIoT exception types with code snippets showing what went wrong.
We can show what happens when CHERI is enabled and when it is disabled.
We will provide at least some of these applications in bare-metal mode where they do not need an operating system.

### Operating system

The [CHERIoT RTOS work](https://github.com/microsoft/cheriot-rtos) will need some reworking in terms of memory layout and drivers to work on the Sonata system.

# FPGA (Soft-Core) architecture

The toolchain for FPGA (hardware/gateware) development is described in the [FPGA Development](fpga-development.md) section.

# PCB architecture

This is described in the [Board](../architecture/board.md) documentation.
