# HyperRAM

HyperRAM is used as an alternative to flash.
Compared to flash, HyperRAM has similar performance but it avoids the need for a quad-speed SPI controller to interact with the flash.
For details on the Windbond W956D8MBYA5I HyperRAM chip used on the Sonata board, see the [datasheet](https://www.winbond.com/resource-files/W956x8MBYA_64Mb_HyperBus_pSRAM_TFBGA24_datasheet_A01-002_20191113.pdf).
We anticipate mostly code to live in HyperRAM and to make sure that we don't suffer from access latency, we enable the instruction cache in Ibex.

## Capability enabled RAM

Currently only 1 MiB of HyperRAM is accessible and all of that has associated capability tags, these tags are stored in 16 KiB of SRAM.
