# Direct memory access (DMA)

DMA is a way to do bulk memory operations without undue burden on the CPU.
The DMA in Sonata is a simple linear source to destination design.
You define a source address, a destination address and number of bytes.
The DMA cannot preserve capability tags across the boundary, so any tags will be lost in this process.
One thing that is important to consider is that a process without a capability to a piece of memory must not be able to use the DMA as a way to gain access to it.
The driver in the operating system must thus enforce that any memory being copied by the DMA is requested by a process that has access to that piece of memory.
If a user cannot find a suitable way to perform this security check in software, they can build an image without the DMA engine present.

To make a capability aware DMA is a bit more complicated and beyond the scope of Sonata.
There is a [position paper](https://www.cl.cam.ac.uk/research/security/ctsrd/pdfs/2020hasp-cheri-dma.pdf) available where a number of different approaches are laid out.

| Offset | Register            |
|--------|---------------------|
| 0x00   | Source address      |
| 0x04   | Destination address |
| 0x08   | Byte count          |
| 0x0C   | Status              |
| 0x10   | Control             |

## Source address

A 32 bit source address.

## Destination address

A 32 bit destination address.

## Byte count

How many bytes to copy from source to destination, only the lowest 20 bits are used.

## Status

| Bit offset | Description |
|------------|-------------|
| 5          | Write error |
| 4          | Read error  |
| 1          | Bus error   |
| 0          | Ready       |

## Control

| Bit offset | Description |
|------------|-------------|
| 5          | Stop        |
| 4          | Disable write |
| 3          | Disable read |
| 2          | Write inc   |
| 1          | Read inc    |
| 0          | Request     |
