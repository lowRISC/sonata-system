# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

adapter driver remote_bitbang

remote_bitbang host localhost
remote_bitbang port 44853

reset_config none

# Configure JTAG chain and the target processor
set _CHIPNAME riscv-cheriot

# Sonata JTAG IDCODE
set _EXPECTED_ID 0x11011CDF

jtag newtap $_CHIPNAME cpu -irlen 5 -expected-id $_EXPECTED_ID -ignore-version
set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME riscv -chain-position $_TARGETNAME

adapter speed 10000

riscv set_mem_access sysbus
gdb_report_data_abort enable
gdb_report_register_access_error enable
gdb_breakpoint_override hard

init
halt
