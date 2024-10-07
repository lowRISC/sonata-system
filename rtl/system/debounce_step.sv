// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Maintain a 1-bit 'counter' that increments when a step-pulse is received
// and the input is in the opposite state from the debounced output.
// If the input remains in that state continuously between one step-pulse
// and the next, it is deemed stable and becomes the debounced output.
// If the input changes (i.e. it is bouncing) we reset the counter.
// The step-pulse should come from a shared counter configured to output
// a one-cycle pulse at a frequency matching the desired debounce period.

module debounce_step (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic step_i,
    input  logic btn_i,
    output logic btn_o
);

  logic cnt_d, cnt_q;
  logic btn_d, btn_q;

  assign btn_o = btn_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin : p_fsm_reg
    if (!rst_ni) begin
      cnt_q <= '0;
      btn_q <= '0;
    end else begin
      cnt_q <= cnt_d;
      btn_q <= btn_d;
    end
  end

  assign btn_d = (step_i & cnt_q) ? btn_i : btn_q;
  // Clear counter-bit if button input equals stored value. Set counter-bit
  // if button input differs from stored value when we receive a step pulse.
  assign cnt_d = (btn_i == btn_q) ? 1'b0 :
                         (step_i) ? 1'b1 :
                                    cnt_q;

endmodule
