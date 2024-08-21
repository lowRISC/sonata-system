## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# This file is for timing constraints to be applied *after* synthesis.
# i.e. timing constraints on internal paths.

set_false_path -from [get_pins -of [get_cells u_sonata_system/g_hyperram.u_hyperram/u_hbmc_tl_top/u_hbmc_cmd_fifo/*storage*/*]]
set_false_path -from [get_pins -of [get_cells u_sonata_system/g_hyperram.u_hyperram/u_hbmc_tl_top/hbmc_ufifo_inst/u_fifo/*storage*/*]]
set_false_path -from [get_pins -of [get_cells u_sonata_system/g_hyperram.u_hyperram/u_hbmc_tl_top/hbmc_dfifo_inst/u_fifo/*storage*/*]]

# TODO: Want some general constraints that will setup appropriate false paths
# for all CDC prims. An attempt is below but it isn't working yet.
#set sync_cells [get_cells -hier -filter {ORIG_REF_NAME == prim_flop_2sync}]
#
#foreach sync_cell $sync_cells {
#  set sync_pins [get_pins -of [get_cells -hier -regexp $sync_cell/.*u_sync_1.*]]
#  if {[info exists endpoint_sync_pins_for_false_paths]} {
#    set endpoint_sync_pins_for_false_paths $sync_pins
#  } else {
#    lappend endpoint_sync_pins_for_false_paths $sync_pins
#  }
#}
#
#set_false_path -to $endpoint_sync_pins_for_false_paths
#
#set async_fifo_cells [get_cells -hier -filter {ORIG_REF_NAME == prim_fifo_async}]
#
#foreach async_fifo_cell $async_fifo_cells {
#  set async_fifo_pins [get_pins -of [get_cells -hier -regexp $async_fifo_cell/.*storage.*]]
#  if {[info exists startpoint_fifo_async_pins_for_false_paths]} {
#    set startpoint_fifo_async_pins_for_false_paths $async_fifo_pins
#  } else {
#    lappend startpoint_fifo_async_pins_for_false_paths $async_fifo_pins
#  }
#}
#
#set_false_path -from $startpoint_fifo_async_pins_for_false_paths
