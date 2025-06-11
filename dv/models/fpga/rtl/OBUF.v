// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a crude model of the Xilinx OBUF primitive; enough to make the
// OpenHBMC implementation simulate correctly.
module OBUF #(
  parameter DRIVE = 0,
  parameter SLEW  = "SLOW"
) (
  input  I,
  output O
);

assign O = I;

endmodule

