# GPIO

| Offset | Register |
|--------|----------|
| 0x00   | Output   |
| 0x04   | Input    |
| 0x08   | Debounced input |

The output register displays the specified value onto the boards output:

| Bit offset | Description |
|------------|-------------|
| 7-0        | LEDs        |

Both input registers:

| Bit offset | Description |
|------------|-------------|
| 13-9       | Joystick (left, down, up, right, press) |
| 8          | Button |
| 7-0        | DIP switches |
