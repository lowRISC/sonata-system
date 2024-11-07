# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Manually add delay to the lcd_copi output path. This is to avoid occasional
# problems when the tool performs all the hold-fixing itself using giant
# hold detours but inserts them before all other paths have branched off,
# leading to internal shift/loopback paths with large setup timing failures.
# Insert logical-buffers (i.e. LUT1 where out=in) just before the port output
# buffer so that the tool does not need to insert huge hold detours elsewhere.
# Each BUF/LUT1 seems to contribute around 0.5-0.9 ns min-delay to hold-fixing,
# but that may change with device or utilisation. Only need to insert
# enough delay to significantly lessen the amount of delay the tool inserts
# elsewhere. No need to be exact, so prefer under-delaying to save LUTs.
set buf_count 15
set port_obuf_cell [get_cells -of [get_nets -of [get_port lcd_copi]]]
set initial_load_pin [get_pins $port_obuf_cell/I]
set initial_driver_pin [get_pins -of [get_nets -of $initial_load_pin] -filter {direction == out}]
disconnect_net -net {lcd_copi_OBUF} -objects [list $initial_driver_pin $initial_load_pin]
for {set i 0} {$i < $buf_count} {incr i} {
  create_cell -reference BUF "del_lcd_copi_$i"
  create_net "n_lcd_copi_$i"
  if {$i == 0} {
	set driver_pin $initial_driver_pin
  } else {
	set driver_pin "del_lcd_copi_[expr {$i - 1}]/O"
  }
  connect_net -hierarchical -net "n_lcd_copi_$i" -objects [list $driver_pin "del_lcd_copi_$i/I0"]
}
create_net "n_lcd_copi_$buf_count"
connect_net -hierarchical -net "n_lcd_copi_$buf_count" -objects [list "del_lcd_copi_[expr {$buf_count - 1}]/O" $initial_load_pin]
# Ensure they do not get optimised away
set_property DONT_TOUCH TRUE [get_cells del_lcd_copi_*]
