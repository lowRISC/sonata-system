// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i2cdpi #(
  // Bus identification.
  parameter string ID = "i2c",
  // Number of Out-Of-Band Input bits.
  parameter int unsigned OOB_InW  = 1,
  // Number of Out-Of-Band Output bits.
  parameter int unsigned OOB_OutW = 1
) (
  input  logic rst_ni,

  // Input signals from the I2C controller.
  input  logic scl_i,
  input  logic sda_i,
  // Outputs to the controller; SCL may be used to implement
  // clock-stretching and SDA provides ACKs and read data.
  output logic scl_o,
  output logic sda_o,

  // Out-Of-Band input/output signals carry additional information if required.
  input  logic [OOB_InW-1:0]  oob_in,
  output logic [OOB_OutW-1:0] oob_out
);

chandle ctx;

// I2C DPI initialisation; DPI model is supplied with the bus identification string and
// can attach devices accordingly.
import "DPI-C" function
  chandle i2cdpi_create(input string name);

// I2C DPI finalisation.
import "DPI-C" function
  void i2cdpi_destroy(input chandle ctx);

// I2C Bus Reset.
import "DPI-C" function
  int unsigned i2cdpi_under_reset(input chandle ctx, int unsigned sda_scl, int unsigned oob_in);

// Decode I2C bus signals.
import "DPI-C" function
  int unsigned i2cdpi_decode(input chandle ctx, int unsigned sda_scl, int unsigned oob_in);

// Initialisation of I2C DPI model.
initial begin
  ctx = i2cdpi_create(ID);
end
// Finalisation of I2C DPI model.
final begin
  i2cdpi_destroy(ctx);
end

// Inform of the I2C DPI model of Bus Reset conditions and I2C signal changes.
logic [31:0] out_q;
always_comb begin
  if (!rst_ni) out_q = i2cdpi_under_reset(ctx, 32'({sda_i, scl_i}), 32'(oob_in));
  else out_q = i2cdpi_decode(ctx, 32'({sda_i, scl_i}), 32'(oob_in));
end

wire unused_ = ^out_q[31:OOB_OutW+2];
assign {oob_out, sda_o, scl_o} = out_q[OOB_OutW+1:0];

endmodule
