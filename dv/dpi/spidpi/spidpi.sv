// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Interfaces to DPI model of SPI device(s).
// - presently this assumes that data is sampled on the rising edge of SCK,
//   so the device launches data on the falling edge.
// - the SPI controller within the Sonata system does support other modes of operation but these
//   are not presently required by the devices that are being simulated.
module spidpi #(
  // Device identification.
  parameter string ID = "generic",
  // Number of SPI devices on the bus (= number of chip selects).
  parameter int unsigned NDevices = 1,
  // Width of COPI and CIPO (= number of data lines).
  parameter int unsigned DataW    = 1,
  // Number of Out-Of-Band Input bits.
  parameter int unsigned OOB_InW  = 1,
  // Number of Out-Of-Band Output bits.
  parameter int unsigned OOB_OutW = 1
) (
  input  logic                rst_ni,

  input  logic                sck,
  /* verilator lint_off SYNCASYNCNET */
  input  logic [NDevices-1:0] cs,
  /* verilator lint_on SYNCASYNCNET */
  input  logic [DataW-1:0]    copi,
  output logic [DataW-1:0]    cipo,

  // Out-Of-Band input/output signals carry additional information if required.
  input  logic [OOB_InW-1:0]  oob_in,
  output logic [OOB_OutW-1:0] oob_out
);

chandle ctx;

// Note: The imported DPI-C functions use 'int unsigned' for their parameters rather than
// parameterising their widths because the module `spidpi` is multiply-instantiated and that would
// lead to Verilator complaining about multiple incompatible signatures for each imported function.

// SPI DPI initialisation; DPI model is supplied with the device identification string and the
// properties of the physical connections.
import "DPI-C" function
  chandle spidpi_create(input string name,      // Device identification string.
                        input int    ndevices,  // Number of devices on bus (= number of selects).
                        input int    dataW,     // Number of data lines.
                        input int    oobIntW,   // Width of Out-Of-Band input data (bits).
                        input int    oobOutW);  // Width of Out-Of-Band output data (bits).
// SPI DPI finalisation.
import "DPI-C" function
  void spidpi_destroy(input chandle ctx);

// Sampling transition occurred on the SCK line.
// - the function is supplied with the new COPI data for a write operation as well as the
//   Out-Of-Band input signals.
import "DPI-C" function
  void spidpi_sampleEdge(input chandle ctx,
                         input int unsigned cs,       // Chip Selects.
                         input int unsigned copi,     // Write data from controller.
                         input int unsigned oob_in);  // Out-Of-Band inputs.

// Launch transition on the SCK lines.
// - the function is supplied with the CS lines and the Out-Of-Band input signals.
// - the result is used to set the new state of the Out-Of-Band output signals (upper bits) and the
//   read data line(s) to the controller (CIPO).
import "DPI-C" function
  bit [OOB_OutW+DataW-1:0] spidpi_launchEdge(input chandle ctx,
                                             input int unsigned cs,       // Chip Selects.
                                             input int unsigned oob_in);  // Out-Of-Band inputs.

// Report a transition on the CS lines; some devices need to be aware of deselection so that an
// ongoing write/read operation may be terminated and a new command accepted.
// - the function is supplied with the new state of the CS lines and the Out-Of-Band input signals.
import "DPI-C" function
  void spidpi_csEdge(input chandle ctx,
                     input int unsigned cs,       // Chip Selects.
                     input int unsigned oob_in);  // Out-Of-Band inputs.

// Initialisation of DPI model.
initial begin
  ctx = spidpi_create(ID, NDevices, DataW, OOB_InW, OOB_OutW);
end
// Finalisation of DPI model.
final begin
  spidpi_destroy(ctx);
end

// Sampling of write data into the device (COPI).
always_ff @(posedge sck or negedge rst_ni) begin
  // Do not invoke the device logic within reset
  if (rst_ni) spidpi_sampleEdge(ctx, 32'(cs), 32'(copi), 32'(oob_in));
end

// Launching of read data from the device (CIPO).
logic [OOB_OutW+DataW-1:0] out_q;
always_ff @(negedge sck or negedge rst_ni) begin
  if (!rst_ni) out_q <= '0;
  else out_q <= spidpi_launchEdge(ctx, 32'(cs), 32'(oob_in));
end

// Report transitions on the CS lines.
always @(cs) begin
  spidpi_csEdge(ctx, 32'(cs), 32'(oob_in));
end

assign {oob_out, cipo} = out_q;

endmodule
