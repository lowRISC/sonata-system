This is a basic testbench to exercise spi_core.
It does not aim to fully verify the design.

To build the testbench a v5 Version of verilator is required (it was testerd with v5.020).

```sh
# Build the simulation
verilator --binary --trace-fst ./spi_core_tb.sv spi_recv.sv spi_trans.sv ../../rtl/spi_core.sv
# Run the simulation
./obj_dir/Vspi_core_tb
```

Any errors will be reported in the simulation output.
The `sim.fst` file contains a complete wave trace of the simulation.
