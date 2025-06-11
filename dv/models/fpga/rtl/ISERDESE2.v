// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a crude model of the Xilinx ISERDESE2 primitive; enough to make the
// OpenHBMC implementation simulate correctly.
module ISERDESE2 #(
  parameter SERDES_MODE       = "MASTER",
  parameter INTERFACE_TYPE    = "NETWORKING",
  parameter DATA_RATE         = "DDR",
  parameter DATA_WIDTH        = 6,
  parameter DYN_CLKDIV_INV_EN = "FALSE",
  parameter DYN_CLK_INV_EN    = "FALSE",
  parameter OFB_USED          = "NONE",
  parameter IOBDELAY          = 0,
  parameter NUM_CE            = 1,
  parameter INIT_Q1           = 1'b0,
  parameter INIT_Q2           = 1'b0,
  parameter INIT_Q3           = 1'b0,
  parameter INIT_Q4           = 1'b0,
  parameter SRVAL_Q1          = 1'b0,
  parameter SRVAL_Q2          = 1'b0,
  parameter SRVAL_Q3          = 1'b0,
  parameter SRVAL_Q4          = 1'b0
) (
  output  O,

  output  Q1,
  output  Q2,
  output  Q3,
  output  Q4,
  output  Q5,
  output  Q6,
  output  Q7,
  output  Q8,

  input   BITSLIP,

  input   CE1,
  input   CE2,

  input   CLK,
  input   CLKB,

  input   CLKDIV,
  input   CLKDIVP,
  input   OCLK,
  input   OCLKB,

  input   D,
  input   DDLY,

  input   OFB,
  input   RST,

  input   DYNCLKDIVSEL,
  input   DYNCLKSEL,

  output  SHIFTOUT1,
  output  SHIFTOUT2,

  input   SHIFTIN1,
  input   SHIFTIN2
);

  reg [8:1] iserdes_int;
  always @(edge CLK or posedge RST) begin
    if (RST) begin
      iserdes_int <= {2{SRVAL_Q4, SRVAL_Q3, SRVAL_Q2, SRVAL_Q1}};
    end else begin
      iserdes_int <= {iserdes_int[7:1], D};
    end
  end

  // In the ISERDESE2 module, Q8 is the oldest bit received.
  assign {Q8,Q7,Q6,Q5,Q4,Q3,Q2,Q1} = iserdes_int;
  assign O = D;

  // These outputs are unused.
  assign {SHIFTOUT2, SHIFTOUT1} = 2'b0;

endmodule

