// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a crude model of the Xilinx ODDR primitive; enough to make the
// OpenHBMC implementation simulate correctly.
module ODDR #(
  parameter DDR_CLK_EDGE = "SAME_EDGE",
  parameter INIT         = 1'b0,
  parameter SRTYPE       = "ASYNC"
) (
  output Q,   // DDR output.
  input  C,   // Clock input.
  input  CE,  // Clock Enable.
  input  D1,  // Two data inputs.
  input  D2,
  input  R,   // ReSet inputs.
  input  S
);

  // Phase detection; PH is asserted during the second (negative edge)
  // phase of the clock 'C.'
  //         ___     ___
  // C   ___/   \___/   \___
  // PH   1   0   1   0   1

  reg PH, PC, NC;
  always @(posedge C) PC <= !PC;
  always @(negedge C) NC <= !NC;
  assign PH = !(PC ^ NC);

  reg S2;
  if (DDR_CLK_EDGE == "SAME_EDGE") begin : gen_same_edge
    // Both data inputs are presented together on posedge.
    always @(posedge C) begin
      if (CE) S2 <= D2;
    end
  end else begin : gen_opposite_edge
    // Inputs are presented on opposite edges.
    assign S2 = D2;
  end

  // Output is clocked on both edges of clock input 'C'.
  reg OUT;
  always @(edge C, posedge R, posedge S) begin
    if (R || S) OUT <= S & ~R;
    else if (CE) OUT <= PH ? D1 : S2;
  end
  assign Q = OUT;

endmodule

