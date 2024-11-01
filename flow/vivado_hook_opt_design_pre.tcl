# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This hook is only run when included in the flow by "vivado_setup.tcl".

# Over-constrain clock used by the core during early implementation stages
# to avoid/reduce post-route timing violations. Clear before route_design.
#
# Over-constraining works by making the tool work a bit harder to optimise
# tight timing paths early on in the implementation flow. This is most
# useful when the tool is under-estimating the future routing delay on
# these paths, as can often happen in congested designs. Over-constraining
# can also help post-route optimisation by improving timing in the wider
# design, giving the optimiser less to fix and more paths that can tolerate
# being adversely modified in order to improve the critical path. Note that
# over-constraining too much can lead to increased runtime and worse timing
# due to over-loading the tool.
set_clock_uncertainty -setup 0.8 [get_clocks clk_sys]
