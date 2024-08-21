## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# This file is for timing constraints to be applied *before* synthesis.
# i.e. timing constraints on top-level ports.

## Clocks
create_clock -period 40.000 -name mainClk -waveform {0.000 20.000} [get_ports mainClk]
create_clock -period 100.000 -name tck_i -waveform {0.000 50.000} [get_ports tck_i]

## Clock Domain Crossings
set clks_sys_unbuf  [get_clocks -of_objects [get_pin u_clkgen/pll/CLKOUT0]]
set clks_usb_unbuf  [get_clocks -of_objects [get_pin u_clkgen/pll/CLKOUT1]]

## Set asynchronous clock groups
set_clock_groups -group ${clks_sys_unbuf} -group ${clks_usb_unbuf} -group mainClk -asynchronous

## HyperRAM
set_false_path -to [get_ports hyperram_ckp]
set_false_path -to [get_ports hyperram_ckn]
set_false_path -to [get_ports hyperram_rwds]
set_false_path -to [get_ports hyperram_dq[*]]

# set input false path. dq[*] and rwds are supposed to
# be fully asynchronous for the data recovery logic
set_false_path -from [get_ports hyperram_rwds]
set_false_path -from [get_ports hyperram_dq[*]]

# False path for 'hb_cs_n' and 'hb_reset_n'
set_false_path -to [get_ports hyperram_cs]
set_false_path -to [get_ports hyperram_nrst]
