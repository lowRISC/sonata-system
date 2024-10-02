# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

adapter driver ftdi
transport select jtag

ftdi_vid_pid 0x0403 0x6011
ftdi_channel 1
ftdi_layout_init 0x0088 0x008b

# Configure JTAG chain and the target processor
set _CHIPNAME riscv

# Sonata JTAG IDCODE
set _EXPECTED_ID 0x11011CDF

jtag newtap $_CHIPNAME cpu -irlen 5 -expected-id $_EXPECTED_ID -ignore-version
set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME riscv -chain-position $_TARGETNAME

adapter speed 15000

riscv set_mem_access sysbus
reset_config none

init
halt
