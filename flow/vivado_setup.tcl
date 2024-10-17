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

# Setup hook scripts, to be called at various stages during the build process
# See Xilinx UG 894 ("Using Tcl Scripting") for documentation.
#
# fusesoc-generated workroot containing the Vivado project file
set workroot [pwd]
# Register custom hooks - must also be added to .core file
# set_property STEPS.SYNTH_DESIGN.TCL.PRE     "${workroot}/vivado_hook_synth_design_pre.tcl"     [get_runs synth_1]
# set_property STEPS.SYNTH_DESIGN.TCL.POST    "${workroot}/vivado_hook_synth_design_post.tcl"    [get_runs synth_1]
set_property STEPS.OPT_DESIGN.TCL.PRE       "${workroot}/vivado_hook_opt_design_pre.tcl"       [get_runs impl_1]
set_property STEPS.OPT_DESIGN.TCL.POST      "${workroot}/vivado_hook_opt_design_post.tcl"      [get_runs impl_1]
# set_property STEPS.PLACE_DESIGN.TCL.PRE     "${workroot}/vivado_hook_place_design_pre.tcl"     [get_runs impl_1]
# set_property STEPS.PLACE_DESIGN.TCL.POST    "${workroot}/vivado_hook_place_design_post.tcl"    [get_runs impl_1]
# set_property STEPS.PHYS_OPT_DESIGN.TCL.PRE  "${workroot}/vivado_hook_phys_opt_design_pre.tcl"  [get_runs impl_1]
# set_property STEPS.PHYS_OPT_DESIGN.TCL.POST "${workroot}/vivado_hook_phys_opt_design_post.tcl" [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.TCL.PRE     "${workroot}/vivado_hook_route_design_pre.tcl"     [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.TCL.POST    "${workroot}/vivado_hook_route_design_post.tcl"    [get_runs impl_1]
# set_property STEPS.WRITE_BITSTREAM.TCL.PRE  "${workroot}/vivado_hook_write_bitstream_pre.tcl"  [get_runs impl_1]
# set_property STEPS.WRITE_BITSTREAM.TCL.POST "${workroot}/vivado_hook_write_bitstream_post.tcl" [get_runs impl_1]
