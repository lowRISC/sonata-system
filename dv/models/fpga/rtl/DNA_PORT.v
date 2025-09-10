// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module DNA_PORT(
  input   CLK,
  input   READ,
  input   SHIFT,
  input   DIN,
  output  DOUT
);

reg [56:0] dna;
always @(posedge CLK) begin
  if (READ) begin
    // This arbitrary number uniquely identifies the simulation model.
    dna <= 57'h1b8a94d_76732894;
  end else if (SHIFT) begin
    // User can extend the DNA value.
    dna <= {dna[55:0], DIN};
  end
end
assign DOUT = dna[56];

endmodule
