# Software architecture

In order to make the Sonata system usable, we need to facilitate running software on it.
We need a compiler toolchain to bootstrap this whole system.

## Toolchain

The toolchain will build on top of the work already done on CHERIoT, which uses a [fork of LLVM](https://github.com/CHERIoT-Platform/llvm-project).
Through Sonata, we are not proposing any changes to the CHERIoT instruction set.
We may need some changes to allow code to be stored in memory that does not have associated tags.

## Applications

Especially for the usable and CHERI-visible requirements, it is important that we have a set of demonstration applications.
One demonstration application is cycling through each of the CHERIoT exception types with code snippets showing what went wrong.
We can show what happens when CHERI is enabled and when it is disabled.
We will provide at least some of these applications in bare-metal mode where they do not need an operating system.

## Operating system

The [CHERIoT RTOS work](https://github.com/microsoft/cheriot-rtos) will need some reworking in terms of memory layout and drivers to work on the Sonata system.
