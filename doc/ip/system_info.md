# System information

This is a special IP block that is automatically populated with useful information about the system.
At the moment it only contains two registers which has the commit hash and the dirty status from the Sonata System repository from which this FPGA bitstream is generated.

| Offset | Register |
|--------|----------|
| 0x0000 | Git commit hash from which this bistream was generated.
| 0x0004 | Git dirty status (1 if dirty or 0 if clean).
