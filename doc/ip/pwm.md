# Pulse width modulation (PWM)

Pulse width modulation allows you to create a block wave with a certain duty cycle.
It is useful for use cases like dimming LEDs.

```text
             <--> pulse-width
___          ____          ____
   |        |    |        |    |           duty-cycle = pulse-width / period
   |________|    |________|    |______
             <--period--->
```

There are 12 PWM channels.

Each channel consists of a free-running 8-bit counter with two configurable comparators.
The period comparator resets the counter value back to zero once it reaches a user-configured top value.
This sets the period (number of cycles between rising-edges of pulses) of the output waveform.
The pulse width comparator changes the channel output from high to low when the counter value surpasses a user-configured pulse width value.
This sets the pulse width (number of cycles the output spends high) of the output waveform.

Each channel has a 64-bit section in the address space for configuration.
Not every channel output has yet been connected to an external pin.

| Offset | PWM channel | External pin |
|--------|-------------|--------------|
| 0x00   | Channel 0   | mikroBUS PWM |
| 0x08   | Channel 1   | N/C          |
| 0x10   | Channel 2   | N/C          |
| 0x18   | Channel 3   | N/C          |
| 0x20   | Channel 4   | N/C          |
| 0x28   | Channel 5   | N/C          |
| 0x30   | Channel 6   | N/C          |
| 0x38   | Channel 7   | N/C          |
| 0x40   | Channel 8   | N/C          |
| 0x48   | Channel 9   | N/C          |
| 0x50   | Channel 10  | N/C          |
| 0x58   | Channel 11  | N/C          |

## Config

For each channel, there is a 32-bit register defining the pulse width and another 32-bit register above it defining how long the complete wave is (counter top).
These registers are write-only, and will return a value of zero if read.

| Offset | Description | Read/Write |
|--------|-------------|------------|
| 0x0    | Pulse width | Write-only |
| 0x4    | Counter top | Write-only |

A channel is enabled by setting an non-zero counter top value.

To generate an always-low signal, set a counter top value of zero.
To generate an always-high signal, set a pulse-width value greater than the (non-zero) counter top value.

