# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This hook is only run when included in the flow by "vivado_setup.tcl".

# Clear clock over-constraint for the routing stage
set_clock_uncertainty -setup 0 [get_clocks clk_sys]
