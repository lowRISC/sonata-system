# Timer

The timer that we use in Sonata is based on [the timer present in the Ibex repository](https://github.com/lowRISC/ibex/blob/master/shared/rtl/timer.sv).
It is a simple memory mapped timer that sends an interrupt to Ibex after a specified amount of time.
The time values in this block are 64 bits, which is why it has a high and a low register for each value.

| Offset | Register |
|--------|----------|
| 0x00   | Time low |
| 0x04   | Time high |
| 0x08   | Time compare low |
| 0x0C   | Time compare high |

The processor can set the time by writing to the time low and high registers and then set a time compare value to compare to.
Internally, the block increments the time by one each clock cycle.
Once the internal counter has exceeded the compare value, it raises an interrupt with the Ibex.
