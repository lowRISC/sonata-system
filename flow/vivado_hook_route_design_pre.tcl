# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Clear clock over-constraint for the routing stage
set_clock_uncertainty -setup 0 [get_clocks clk_sys]
