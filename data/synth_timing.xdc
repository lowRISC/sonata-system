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

## prim_flop_2sync
# Set false_path timing exceptions on 2-stage synchroniser inputs.
# Target the inputs because the flops inside are clocked by the destination.
#
# Reliant on the hierarchical pin names of the synchronisers remaining
# unchanged during synthesis due to use of DONT_TOUCH or KEEP_HIERARCHY.
set sync_cells [get_cells -hier -filter {ORIG_REF_NAME == prim_flop_2sync}]
set sync_pins [get_pins -filter {REF_PIN_NAME =~ d_i*} -of $sync_cells]
# Filter out any that do not have a real timing path (fail to find leaf cell).
set sync_endpoints [filter [all_fanout -endpoints_only -flat $sync_pins] IS_LEAF]
set_false_path -to $sync_endpoints

## prim_fifo_async and prim_fifo_async_simple
# Set false_path timing exceptions on asynchronous fifo outputs.
# Target the outputs because the storage elements are clocked by the source
# clock domain (but made safe to read from the destination clock domain
# thanks to the gray-coded read/write pointers and surrounding logic).
#
# Reliant on the hierarchical pin names of the async fifos remaining
# unchanged during synthesis due to use of DONT_TOUCH or KEEP_HIERARCHY.
set async_fifo_cells [get_cells -hier -regexp -filter {ORIG_REF_NAME =~ {prim_fifo_async(_simple)?}}]
set async_fifo_pins [get_pins -filter {REF_PIN_NAME =~ rdata_o*} -of $async_fifo_cells]
set async_fifo_startpoints [all_fanin -startpoints_only -flat $async_fifo_pins]
# Specify `-through` as well as `-from` to avoid including non-rdata_o paths,
# such as paths from the read pointers that stay internal or exit via rvalid_o.
set_false_path -from $async_fifo_startpoints -through $async_fifo_pins
