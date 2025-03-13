# Debug module

The debug module we use in Sonata is not a regular RISC-V debug module, but one that is CHERIoT-enabled.
The source of the debug module can be found [here](https://github.com/CHERIoT-Platform/cheriot-dbg-module).

## FPGA

You can connect to the debug module on the Sonata board by running OpenOCD directly:
```sh
openocd -f util/sonata-openocd-cfg.tcl
```

## JTAG DPI

We also have support for JTAG DPI so you can debug a simulation.

For example in one terminal you can run:
```sh
build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator -t -E sw/cheri/build/checks/uart_check
```
Then in another terminal you can run:
```sh
openocd -f util/verilator-openocd-cfg.tcl
```

## GDB quick guide

To connect to the OpenOCD instance you should run GDB in a separate terminal while OpenOCD is running in another:
```sh
# We recommend using the lowRISC toolchain:
# https://github.com/lowRISC/lowrisc-toolchains/releases
riscv32-unknown-elf-gdb
```

When that starts up you should be greeted with a `(gdb)` prompt.
To connect to your OpenOCD instance, independent of whether this is on FPGA or Verilator, you should use the following command:
```
(gdb) target extended-remote localhost:3333
```

Now there are a number of useful commands you can do, for example you can read out the git hash value of the system you are running:
```
(gdb) x/x 0x8000C000
```

You can also inspect the values of all the registers in the system:
```
(gdb) info all-r
```

You can set a breakpoint using a program counter value which you can find using a disassembled ELF.
You can then run the program from the beginning:
```
(gdb) break *0x00100080
Breakpoint 1 at 0x100080

(gdb) run
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program:

Breakpoint 1, 0x00100080 in ?? ()

(gdb) continue
Continuing.
```

