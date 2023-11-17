# HyperRAM controller

HyperRAM is used as an alternative to flash.
Compared to flash, HyperRAM has similar performance but it avoids the need for a quad-speed SPI controller to interact with the flash.
The HyperRAM controller is the interface between the Sonata system and the actual chip.

| Offset | Register        |
|--------|-----------------|
| 0x00   | Configuration 0 |
| 0x04   | Configuration 1 |

For details of what these configuration registers do please consult Section 9.4 and 9.5 of the [datasheet](https://www.mouser.co.uk/datasheet/2/949/W956x8MBYA_64Mb_HyperBus_pSRAM_TFBGA24_datasheet_A-1760356.pdf).

Because the latency of accessing data memory through the HyperRAM will be quite slow, we introduce a fully-associated cache of a few words to improve performance.
It is anticipated main data storage will be in SRAM with the HyperRAM storing small amounts of data interleaved with code so more significant caching is unnecessary.

## Capability enabled RAM

The HyperRAM controller is also where the capability tags live for the tagged part of the RAM.
In Sonata we allocate 32 KiB for capability tags for RAM which translates in to 1 MiB for tagged RAM.
