# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Some helpful TCL procedures for use in an interactive Vivado session.
# Activate when launching Vivado using:
#   vivado -source util/interactive_helpers.tcl ...
# or later in the Vivado TCL shell using:
#   source util/interactive_helpers.tcl

# Change Selection
proc cs args {select_objects {*}$args}

# TIMing report - setup
proc tim   args {                report_timing   -delay_type max {*}$args}
# TIMing report - Hold
proc timh  args {                report_timing   -delay_type min {*}$args}
# select TIMing Path - setup
proc timp  args {select_objects [get_timing_path -delay_type max {*}$args]}
# select TIMing Path - Hold
proc timph args {select_objects [get_timing_path -delay_type min {*}$args]}

# HighLight objects
proc  hl args {  highlight_objects {*}$args}
# UnHighLight objects (all if no args given)
proc uhl args {unhighlight_objects {*}$args}

# Advanced multi-group highlighting
#
# Vivado default colour index mapping:
#   1: magenta (RGB)      2: yellow        3: bright green     4: blue
#   5: violet             6: gold          7: red              8: cyan (RGB)
#   9: magenta (again)   10: lavender      11: forest green    12: yellow-green
#  13: purple            14: brown         15: raspberry red   16: turquoise
#  17: plum              18: gold (again)  19: blue (again)    20: dark blue
set hlg_default_groups {
  {*ibex_core* 7} {*hardware_revoker* 13} {*cheri_stkz* 5} {*plic* 17} {*rv_timer* 1}
  {*register_file* 15} {*tag_bank* 13} {*u_sram_top* 20} {*hyperram* 4}
  {*pinmux* 6} {*gpio* 18} {*usbdev* 12} {*uart* 16} {*spi* 3} {*i2c* 11} {*pwm* 2}
  {*dm_top* 14} {*xbar* 10}
}
# HighLight Groups.
# Highlight sequential cells various colours based on the groups specified.
# Each group is defined by a register name pattern and colour index.
# The default groups are defined above and will be used if called without args.
# Optionally highlight a proportion of the combinatorial fanin/fanout
# (takes ages and ruins overview, but good for detailed work), e.g. 0.05.
proc hlg [list [list name_colour_pairs $hlg_default_groups] [list comb_prob 0.00]] {
  unhighlight_objects
  set used_colours {}
  foreach name_and_colour $name_colour_pairs {
    lassign $name_and_colour name colour
    # If no colour index specified, find lowest unused colour index
    if {$colour == {}} {
      for {set colour 1} {[lsearch $used_colours $colour]!=-1} {incr colour} {}
    }
    lappend used_colours $colour
    # Highlight sequential cells (SLICE registers, block RAMs)
    set sequ [get_cells -hier -filter "NAME =~ $name && IS_SEQUENTIAL"]
    highlight_objects -color_index $colour $sequ
    # Highlight a proportion of combinatorial fanin/fanout cells if requested
    if {$comb_prob > 0} {
      set endpoints   [get_pins -of $sequ -filter {DIRECTION == IN && !IS_CLOCK}]
      set startpoints [get_pins -of $sequ -filter {IS_CLOCK}]
      set fanin  [all_fanin  -flat -only_cells $endpoints]
      set fanout [all_fanout -flat -only_cells $startpoints]
      set comb [filter [concat $fanin $fanout] {IS_PRIMITIVE && !IS_SEQUENTIAL}]
      set comb_subset {}
      foreach c $comb {if {rand() < $comb_prob} {lappend comb_subset $comb}}
      highlight_objects -color_index $colour $comb_subset
    }
  }
}
