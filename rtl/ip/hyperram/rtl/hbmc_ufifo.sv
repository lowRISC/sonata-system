// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Reimplementation of hbmc_ufifo using OpenTitan primitives, only works for DATA_WIDTH == 32

module hbmc_ufifo #
(
    parameter integer DATA_WIDTH = 32
)
(

    input   wire                        fifo_wr_clk,
    input   wire                        fifo_wr_nrst,
    input   wire    [15:0]              fifo_wr_din,
    input   wire                        fifo_wr_last,
    input   wire                        fifo_wr_ena,
    output  wire                        fifo_wr_full,

    input   wire                        fifo_rd_clk,
    input   wire                        fifo_rd_nrst,
    output  wire    [DATA_WIDTH - 1:0]  fifo_rd_dout,
    output  wire    [9:0]               fifo_rd_free,
    output  wire                        fifo_rd_last,
    input   wire                        fifo_rd_ena,
    output  wire                        fifo_rd_empty
);
  // FIFO contains 32-bit data word and 1 'last' bit
  localparam int unsigned FIFOWidth = DATA_WIDTH + 1;

  logic [FIFOWidth-1:0] fifo_wdata, fifo_rdata;
  logic [15:0]          fifo_wdata_first_half;

  logic fifo_wready, fifo_wvalid, fifo_rvalid;
  logic fifo_wdata_half_sel;

  assign fifo_wr_full  = ~fifo_wready;
  assign fifo_rd_empty = ~fifo_rvalid;

  assign fifo_wdata = {fifo_wr_last, fifo_wr_din, fifo_wdata_first_half};
  assign fifo_wvalid = fifo_wdata_half_sel & fifo_wr_ena;

  always @(posedge fifo_wr_clk or negedge fifo_wr_nrst) begin
    if (!fifo_wr_nrst) begin
      fifo_wdata_half_sel <= 1'b0;
    end else begin
      if (fifo_wr_ena) begin
        if (!fifo_wdata_half_sel) begin
          fifo_wdata_first_half <= fifo_wr_din;
        end

        fifo_wdata_half_sel <= ~fifo_wdata_half_sel;
      end
    end
  end

  prim_fifo_async #(
    .Width(FIFOWidth),
    .Depth(4)
  ) u_fifo (
    .clk_wr_i(fifo_wr_clk),
    .rst_wr_ni(fifo_wr_nrst),
    .wvalid_i(fifo_wvalid),
    .wready_o(fifo_wready),
    .wdata_i(fifo_wdata),
    .wdepth_o(),

    .clk_rd_i(fifo_rd_clk),
    .rst_rd_ni(fifo_rd_nrst),
    .rvalid_o(fifo_rvalid),
    .rready_i(fifo_rd_ena),
    .rdata_o(fifo_rdata),
    .rdepth_o()
  );

  // fifo_rd_free output is unused in hyperram top-level
  assign fifo_rd_free = '0;

  assign fifo_rd_dout = fifo_rdata[31:0];
  assign fifo_rd_last = fifo_rdata[32];

  initial begin
    if (DATA_WIDTH != 32) begin
      $fatal("hbmc_ufifo only supports DATA_WIDTH of 32");
    end
  end

endmodule

/*----------------------------------------------------------------------------------------------------------------------------*/

`default_nettype wire
