// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Produces asynchronous-assert synchronous-deassert resets for various
// clock domains from the input main reset signal.

module rst_sync (
  input clk_sys_i,
  input clk_usb_i,

  input rst_ni,

  output rst_sys_no,
  output rst_usb_no
);
  // clk_sys
  prim_flop_2sync #(
    .Width(1),
    .ResetValue('0)
  ) u_clk_sys_sync (
    .clk_i(clk_sys_i),
    .rst_ni(rst_ni),
    .d_i(1'b1),
    .q_o(rst_sys_no)
  );
  // clk_usb
  prim_flop_2sync #(
    .Width(1),
    .ResetValue('0)
  ) u_clk_usb_sync (
    .clk_i(clk_usb_i),
    .rst_ni(rst_ni),
    .d_i(1'b1),
    .q_o(rst_usb_no)
  );
endmodule
