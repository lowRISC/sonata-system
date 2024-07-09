# CHERIoT hardware revocation engine

This block is integrated from the [CHERIoT Safe repository](https://github.com/microsoft/cheriot-safe/blob/main/src/msft_cheri_subsystem/msftDvIp_mmreg.sv).
It controls the hardware revoker inside CHERIoT Ibex which is the thing that does the revocation, it is looking for capabilities to freed memory.
When a piece of memory is freed its revocation bits are set.
The hardware revoker sweeps from the start until the end address for capabilities that point to any revoked region.
For more information on how this works please refer to [Section 3.3.3 (background pipelined revoker) of this paper](https://cheriot.org/papers/2023-micro-cheriot-uarch.pdf).

These are the register offsets:

| Offset | Register |
|--------|----------|
| 0x0000 | Start address to begin sweeping.
| 0x0004 | End address to stop sweeping.
| 0x0008 | Most significatn byte is set to constant `0x55`. least significant bit is asserted for one cycle when sweeping starts.
| 0x000C | Epoch, where the last bit means a sweep is currently happening.
| 0x0010 | Read interrupt status, but is only high for one cycle.
| 0x0014 | Enable bit for raising an interrupt when sweeping is done.
| 0x0040 | Debug: FIFO empty and read data values.
| 0x0044 | Debug FIFO full, empty and depth values.
