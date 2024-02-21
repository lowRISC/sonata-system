Boot test for Sonata system
==========================

This contains the code necessary to test pure capability mode on the Sonata system:
- Store data (by changing LEDs)
- Load data (by reading switches)
- Store capability to SRAM
- Load capability from SRAM
This produces a `cpu0_irom.vmem` file that should be baked into the bitfile.


Building
--------

This is built with a simple Makefile that expects to be passed the locations of various tools and the RTOS directory as part of the make invocation:

```
$ make CHERIOT_LLVM_ROOT=path/to/llvm/bin/ CHERIOT_RTOS_SDK=/path/to/cheriot-rtos/sdk/
```

