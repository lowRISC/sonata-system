## Copyright lowRISC contributors.
## Licensed under the Apache License, Version 2.0, see LICENSE for details.
## SPDX-License-Identifier: Apache-2.0

# Sonata XL-specific, to be sourced after synth_timing_common.xdc

# This file is for timing constraints to be applied *before* synthesis.
# i.e. timing constraints on top-level ports.
#
# See UG949 and UG903 for information on setting various timing constraints.

#### Recommended timing constraints sequence from UG949 ####
## Timing Assertions Section
# Primary clocks
# Virtual clocks
# Generated clocks
# Delay for external MMCM/PLL feedback loop
# Clock Uncertainty and Jitter
# Input and output delay constraints
# Clock Groups and Clock False Paths
## Timing Exceptions Section
# False Paths
# Max Delay / Min Delay
# Multicycle Paths
# Case Analysis
# Disable Timing

### Clock Groups and Clock False Paths ###
# JTAG tck is completely asynchronous to FPGA mainClk and
# internal clocks generated from it (and thus synchronous with it).
# No HyperRAM on Sonata XL, so exclude HyperRAM clocks.
set_clock_groups -asynchronous -group tck -group {mainClk clk_sys clk_usb}
