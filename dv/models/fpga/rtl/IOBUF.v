// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a crude model of the Xilinx IOBUF primitive; enough to make the
// OpenHBMC implementation simulate correctly.
module IOBUF #(
  parameter DRIVE = 0,
  parameter SLEW  = "SLOW"
) (
  output O,
  inout  IO,
  input  I,
  input  T
);

assign O = IO;
assign IO = T ? 1'bZ : I;

endmodule

