# Direct memory access (DMA)

DMA is a way to do bulk memory operations without undue burden on the CPU.
The DMA in Sonata is a simple linear source to destination design.
You define a source address, a destination address and number of bytes.
The DMA cannot preserve capability tags across the boundary, so any tags will be lost in this process.
One thing that is important to consider is that a process without a capability to a piece of memory must not be able to use the DMA as a way to gain access to it.

This DMA controller will check the capability tag and do bounds and permission checking before performing an operation.

| Offset | Register                    |
|--------|-----------------------------|
| 0x00   | Source capability high      |
| 0x04   | Source capability low       |
| 0x08   | Destination capability high |
| 0x0C   | Destination capability low  |
| 0x10   | Byte count                  |
| 0x14   | Status                      |
| 0x18   | Control                     |

## Source and destination capabilities

The high part of these capabilities are actually 33 bit registers and they take over the capability tag bit that is present on the bus.
The rest of the high part includes the permissions, seal and bounds.

The low part of these capabilities contain a 32 bit address.

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
