# Pulse width modulation (PWM)

Pulse width modulation allows you to create a block wave with a certain duty cycle.
It is useful for use cases like dimming LEDs.

| Offset | Register |
|--------|----------|
| 0x00   | Enable   |
| 0x10   | Config 0 |
| 0x14   | Config 1 |
| 0x18   | Config 2 |
| 0x1C   | Config 3 |
| 0x20   | Config 4 |
| 0x24   | Config 5 |
| 0x28   | Config 6 |
| 0x2C   | Config 7 |

## Enable

This has a bit per PWM on whether to enable it or not.

| Bit offset | Description      |
|------------|------------------|
| 7          | Enable for PWM 7 |
| ...        | ...              |
| 1          | Enable for PWM 1 |
| 0          | Enable for PWM 0 |

## Config

For each output, there is a register defining the pulse width and how long the complete wave is (indicated by the counter value).

| Bit offset | Description |
|------------|-------------|
| 31-16      | Counter     |
| 15-0       | Pulse width |
