# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

# Optional post-route optimisation
if {$opt >= 1} {
  # Post-route optimisation that can fix small (~0.6 ns) timing failures
  # at the cost of up to a few minutes additional runtime.
  #
  # From UG904: RuntimeOptimized
  #   Provides a reduced set of physical optimizations with the shortest
  #   runtime. Use this directive when compile time reduction is more important
  #   than design performance. RuntimeOptimized includes fanout_opt,
  #   critical_cell_opt, placement_opt, and bram_enable_opt.
  phys_opt_design -directive RuntimeOptimized
}
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
  phys_opt_design -directive AggressiveExplore
}
