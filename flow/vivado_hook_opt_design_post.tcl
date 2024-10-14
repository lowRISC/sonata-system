# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Optional additional logic optimisation
if {$opt >= 2} {
  # Perform further logic optimisation to improve the timing fixing ability
  # of post-route phys_opt_design optimisation. May make timing worse
  # before post-route optimisation hopefully makes it better.
  opt_design -directive ExploreWithRemap
}
