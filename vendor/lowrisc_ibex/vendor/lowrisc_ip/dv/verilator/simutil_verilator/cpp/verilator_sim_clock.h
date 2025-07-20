// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENTITAN_HW_DV_VERILATOR_SIMUTIL_VERILATOR_CPP_VERILATOR_SIM_CLOCK_H_
#define OPENTITAN_HW_DV_VERILATOR_SIMUTIL_VERILATOR_CPP_VERILATOR_SIM_CLOCK_H_
#include <cassert>
#include <cstdint>

#include "verilated_toplevel.h"

/**
 * Clock signal in Verilated top level.
 */
class VerilatorSimClock {
 public:
  /**
   * Initialize a clock signal with its properties.
   * All time parameters are specified in picoseconds for accuracy.
   *
   * @param sig_clk Signal in the Verilated top level object.
   * @param hi_interval Time interval for the 'hi' phase of the clock, in ps.
   * @param lo_interval Time interval for the 'lo' phase of the clock, in ps.
   * @param init_offset Time interval until the first transition of the clock, in ps.
   * @param hi_jitter Amount by which 'hi_interval' may vary, in picoseconds.
   * @param lo_jitter Amount by which 'lo_interval' may vary, in picoseconds.
   */
  VerilatorSimClock(CData *sig_clk, uint32_t hi_interval, uint32_t lo_interval,
                    uint32_t init_offset = 0u, uint32_t hi_jitter = 0u, uint32_t lo_jitter = 0u)
      : sig_clk_(sig_clk),
        hi_interval_(hi_interval),
        lo_interval_(lo_interval),
        hi_jitter_(hi_jitter),
        lo_jitter_(lo_jitter),
        lfsr_(1u),
        del_(init_offset) {
    // Each clock interval (hi/lo) is permitted to vary in the range
    // (-jitter/2, +jitter/2)
    assert(hi_jitter < hi_interval && lo_jitter < lo_interval);
  }

  /**
   * Ascertain the remaining time until the next transition of this clock
   * signal.
   *
   * @return Time in picoseconds.
   */
  uint32_t EdgeTimeDelta() const { return del_; }

  /**
   * Advance the state of this clock by the specified number of elapsed
   * picoseconds, modifying the clock signal when a transition occurs.
   *
   * @return true iff a positive (rising) edge of this clock signal occurred.
   */
  bool PosEdgeAfterTime(uint32_t elapsed) {
    // Transition occurred?
    if (elapsed < del_) {
      del_ -= elapsed;
      return false;
    }

    // We should not be advancing beyond a single clock transition or the
    // scheduling failed.
    assert(elapsed == del_);

    // Remember and invert the state of the controlled signal.
    bool was_lo = !*sig_clk_;
    *sig_clk_ = was_lo;
    if (was_lo) {
      // Starting hi interval.
      del_ = (uint32_t)((int32_t)hi_interval_ + Randomize(hi_jitter_));
    } else {
      del_ = (uint32_t)((int32_t)lo_interval_ + Randomize(lo_jitter_));
    }
    return was_lo;
  }

 private:
  int32_t Randomize(int32_t jitter) {
    if (jitter) {
      // lfsr_ cannot be zero (isolated state in sequence).
      // Use (-50,+50%) of the specified jitter, uniform distribution.
      jitter = (jitter * ((int32_t)lfsr_ - 128)) / 254;

      // Simple LFSR for 8-bit sequences
      lfsr_ = (uint8_t)((uint8_t)((lfsr_) << 1) ^
                        ((((lfsr_) >> 1) ^ ((lfsr_) >> 2) ^ ((lfsr_) >> 3) ^
                          ((lfsr_) >> 7)) &
                         1U));
    }
    return jitter;
  }

  // Controlled signal in Verilated top level.
  CData *sig_clk_;

  // All times are in picoseconds.
  uint32_t hi_interval_;  // Mean time for which the clock is high
  uint32_t lo_interval_;  // Mean time low
  uint32_t hi_jitter_;    // Amount by which the hi interval may jitter
  uint32_t lo_jitter_;

  // Randomization using LFSR.
  uint8_t lfsr_;

  // Time remaining until transition.
  uint32_t del_;
};

#endif  // OPENTITAN_HW_DV_VERILATOR_SIMUTIL_VERILATOR_CPP_VERILATOR_SIM_CLOCK_H_
