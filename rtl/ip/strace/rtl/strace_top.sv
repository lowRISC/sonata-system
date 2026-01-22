// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Sonata Trace Port functionality.
module strace_top
#(
  // Number of I2C buses to trace.
  parameter int unsigned I2C_NUM = 1,
  // Number of SPI buses to trace.
  parameter int unsigned SPI_NUM = 1
) (
  input  logic               clk_i,
  input  logic               rst_ni,

  // Configuration interface.
  input  logic               cfg_re,
  input  logic               cfg_we,
  input  logic [3:0]         cfg_addr,
  input  logic [31:0]        cfg_wdata,
  output logic [31:0]        cfg_rdata,

  // I2C bus(es).
  input  logic [I2C_NUM-1:0] i2c_scl,
  input  logic [I2C_NUM-1:0] i2c_sda,

  // SPI bus(es).
  input  logic [SPI_NUM-1:0] spi_cs,
  input  logic [SPI_NUM-1:0] spi_sck,
  input  logic [SPI_NUM-1:0] spi_copi,
  input  logic [SPI_NUM-1:0] spi_cipo,

  // TL-UL activity.
  input  logic [3:0]         tlul_trace,

  // Trace output.
  output logic [3:0]         strace_o
);

// TODO: We probably want to use structured data types to prevent awkward reordering of signals.

// Re-time all of the inputs by running them through synchronizers.
// TODO: Simplify this by padding to 4 here?
localparam int unsigned TraceInW = 4 + SPI_NUM*4 + I2C_NUM*2;
logic [TraceInW-1:0] trace_in;
prim_flop_2sync #(
  .Width      (TraceInW)
) u_cdc (
  .clk_i      (clk_i),
  .rst_ni     (rst_ni),
  .d_i        ({tlul_trace,
                spi_cipo, spi_copi, spi_sck, spi_cs,
                i2c_sda, i2c_scl}),
  .q_o        (trace_in)
);

// Instantiate the execution trace logic.

// The configuration interface presently consists of nothing more than a control register.
reg [31:0] control;
wire       enable  = control[31];   // Enabling trace output?
wire [3:0] src_sel = control[3:0];  // Source selection.
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
// TODO: For now we want the strace to be on the microSD card output.
//    control <= 32'b0;
    control <= 32'h8000_0000 | 32'(I2C_NUM);
  end else if (cfg_we) begin
    case (cfg_addr)
      default: control <= cfg_wdata;
    endcase
  end
end
assign cfg_rdata = control;  // There's only a single register at present.

// We may now choose the appropriate source.
localparam int unsigned SPI_OFST = I2C_NUM * 2;
logic [4:0] spi_idx = 5'(32'(src_sel) - I2C_NUM);
logic [3:0] sel_strace;
always_comb begin
  if (32'(src_sel) < I2C_NUM) begin
    sel_strace = {2'b00, trace_in[I2C_NUM + 32'(src_sel)], trace_in[{1'b0,src_sel}]};
  end else if (32'(src_sel) < I2C_NUM + SPI_NUM) begin : sel_spi
    sel_strace = {trace_in[SPI_NUM*3 + SPI_OFST + 32'(spi_idx)],
                  trace_in[SPI_NUM*2 + SPI_OFST + 32'(spi_idx)],
                  trace_in[SPI_NUM   + SPI_OFST + 32'(spi_idx)],
                  trace_in[            SPI_OFST + 32'(spi_idx)]};
  end else begin
    sel_strace = trace_in[SPI_NUM*4 + I2C_NUM*2+3 -: 4];
  end
end

// Output the trace signals.
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
    strace_o  <= 4'b0;
  end else if (enable) begin
    strace_o  <= sel_strace;
  end
end

logic unused_;
assign unused_ = cfg_re;

endmodule : strace_top
