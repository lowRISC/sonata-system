# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

## Vivado project custom configuration script
##
## For configuration not supported by fusesoc/edalize

# Configure when and how the post-synthesis timing XDC file is read.
# The `Tcl` `file_type` makes Vivado use the `-unmanaged` argument with
# `read_xdc`, which allows us to use commands such as `foreach`.
# The `used_in_synthesis` property makes Vivado delay reading the file
# until after synthesis, allowing internal paths to be specified.
# Note: `file_type` must be set before `used_in_synthesis`.
set_property file_type Tcl [get_files impl_timing.xdc]
set_property used_in_synthesis false [get_files impl_timing.xdc]

# Set optimisation level for use in this hook and others later on.
# - Set `opt` to `0` for no extra optimisation (Vivado default).
# - Set `opt` to `1` for light & effective extra optimisation (general use).
# - Set `opt` to `2` for heavy & chancy extra optimisation (short-term boost).
set opt 1

# Setup hook scripts, to be called at various stages during the build process
# See Xilinx UG 894 ("Using Tcl Scripting") for documentation.
#
# fusesoc-generated workroot containing the Vivado project file
set workroot [pwd]
# Register custom hooks - must also be added to .core file
# set_property STEPS.SYNTH_DESIGN.TCL.PRE     "${workroot}/vivado_hook_synth_design_pre.tcl"     [get_runs synth_1]
# set_property STEPS.SYNTH_DESIGN.TCL.POST    "${workroot}/vivado_hook_synth_design_post.tcl"    [get_runs synth_1]
if {$opt >= 1} {
  # Set extra clock margin for early implementation stages
  # (cleared in STEPS.ROUTE_DESIGN.TCL.PRE).
  set_property STEPS.OPT_DESIGN.TCL.PRE       "${workroot}/vivado_hook_opt_design_pre.tcl"       [get_runs impl_1]
}
# set_property STEPS.OPT_DESIGN.TCL.POST      "${workroot}/vivado_hook_opt_design_post.tcl"      [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.TCL.PRE     "${workroot}/vivado_hook_place_design_pre.tcl"     [get_runs impl_1]
# set_property STEPS.PLACE_DESIGN.TCL.POST    "${workroot}/vivado_hook_place_design_post.tcl"    [get_runs impl_1]
# set_property STEPS.PHYS_OPT_DESIGN.TCL.PRE  "${workroot}/vivado_hook_phys_opt_design_pre.tcl"  [get_runs impl_1]
# set_property STEPS.PHYS_OPT_DESIGN.TCL.POST "${workroot}/vivado_hook_phys_opt_design_post.tcl" [get_runs impl_1]
if {$opt >= 1} {
  # Clear extra clock margin following early implementation stages
  # (set in STEPS.OPT_DESIGN.TCL.PRE).
  set_property STEPS.ROUTE_DESIGN.TCL.PRE     "${workroot}/vivado_hook_route_design_pre.tcl"     [get_runs impl_1]
}
# set_property STEPS.ROUTE_DESIGN.TCL.POST    "${workroot}/vivado_hook_route_design_post.tcl"    [get_runs impl_1]
# set_property STEPS.WRITE_BITSTREAM.TCL.PRE  "${workroot}/vivado_hook_write_bitstream_pre.tcl"  [get_runs impl_1]
# set_property STEPS.WRITE_BITSTREAM.TCL.POST "${workroot}/vivado_hook_write_bitstream_post.tcl" [get_runs impl_1]

# Enable or tweak existing Vivado optimisation stages
if {$opt >= 2} {
  # Perform further logic optimisation to improve the timing fixing ability
  # of post-route phys_opt_design optimisation. May make timing worse
  # before post-route optimisation hopefully makes it better.
  set opt_design_args {; opt_design -directive ExploreWithRemap}
  set_property -name {STEPS.OPT_DESIGN.ARGS.MORE OPTIONS} -value $opt_design_args -object [get_runs impl_1]
}
if {$opt >= 1} {
  set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED TRUE [get_runs impl_1]
  # Post-route optimisation that can fix small (~0.6 ns) timing failures
  # at the cost of up to a few minutes additional runtime.
  #
  # From UG904: RuntimeOptimized
  #   Provides a reduced set of physical optimizations with the shortest
  #   runtime. Use this directive when compile time reduction is more important
  #   than design performance. RuntimeOptimized includes fanout_opt,
  #   critical_cell_opt, placement_opt, and bram_enable_opt.
  set post_route_args {-directive RuntimeOptimized}
  if {$opt >= 2} {
    # Additional post-route optimisation that can help fix any remaining
    # timing failures at the cost of up to several minutes additional runtime.
    #
    # From UG904: AggressiveExplore
    #   Similar to Explore but with different optimization algorithms and
    #   more aggressive goals. Includes a SLR crossing optimization phase that
    #   is allowed to degrade WNS which should be regained in subsequent
    #   optimization algorithms. Also includes a hold violation fixing
    #   optimization.
    # Appears to use different algorithm/optimisations from RuntimeOptimized.
    set post_route_args [concat $post_route_args {; phys_opt_design -directive AggressiveExplore}]
  }
  set_property -name {STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.MORE OPTIONS} -value $post_route_args -object [get_runs impl_1]
}
