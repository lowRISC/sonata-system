# I2C host

| Offset | Register |
|--------|----------|
| 0x00   | Reserved |
| 0x04   | Status   |
| 0x08   | Read data |
| 0x0C   | Format data |
| 0x10   | FIFO control |
| 0x14   | FIFO status |
| 0x18   | Reserved |
| 0x1C   | Reserved |
| 0x20   | Timing low and high |
| 0x24   | Timing fall and rise |
| 0x28   | Timing start and repeated start |
| 0x2C   | Timing hold and setup |
| 0x30   | Timing buffer and stop |
| 0x34   | Stretch timeout |
| 0x38   | Reserved |
| 0x3C   | Reserved |
| 0x40   | Reserved |
| 0x44   | Clock timeout |

Status:

| Bit offset  | Description |
|-------------|-------------|
| 9           | Acquire FIFO empty |
| 8           | Transmit FIFO empty |
| 7           | Acquire FIFO full |
| 6           | Transmit FIFO full |
| 5           | Receive FIFO empty |
| 4           | Reserved |
| 3           | Host idle |
| 2           | Format FIFO empty |
| 1           | Receive FIFO full |
| 0           | Format FIFO full |

Read data:

| Bit offset  | Description |
|-------------|-------------|
| 7-0         | Read data   |

Format data:

| Bit offset  | Description |
|-------------|-------------|
| 12          | Don't acknowledge current byte |
| 11          | Continue read after last byte |
| 10          | Read format byte number of bytes (256 if format byte is zero) |
| 9           | Issue stop after this operation |
| 8           | Issue start before byte |
| 7-0         | Format data |

FIFO control:

| Bit offset  | Description |
|-------------|-------------|
| 8-2         | Reserved    |
| 1           | Format FIFO reset |
| 0           | Receive FIFO reset |

FIFO status:

| Bit offset  | Description |
|-------------|-------------|
| 30-24       | Reserved    |
| 22-16       | Receive FIFO fill level |
| 14-8        | Reserved |
| 6-0         | Format FIFO fill level |

Timing low and high:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold SCL low |
| 15-0        | Time to hold SCL high |

Timing fall and rise:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Anticipated bus fall time |
| 15-0        | Anticipated bus rise time |

Timing start and repeated start:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold start signals |
| 15-0        | Setup time for repeated start signals |

Timing hold and setup:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time to hold data and acknowledge bits |
| 15-0        | Setup time for data and acknowledge bits |

Timing buffer and stop:

| Bit offset  | Description |
|-------------|-------------|
| 31-16       | Time between each stop signal and following start signal |
| 15-0        | Setup time for stop signals |

Stretch timeout control:

| Bit offset  | Description |
|-------------|-------------|
| 31          | Enable timeout |
| 30-0        | Timeout value |

Clock timeout control:

| Bit offset | Description |
|------------|-------------|
| 31-0       | Host clock generation timeout value |
