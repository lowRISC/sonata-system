// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Reimplementation of hbmc_dfifo using OpenTitan primitives, only works for DATA_WIDTH == 32
module hbmc_dfifo #
(
    parameter integer DATA_WIDTH = 32
)
(
    input   wire                            fifo_wr_clk,
    input   wire                            fifo_wr_nrst,
    input   wire    [DATA_WIDTH - 1:0]      fifo_wr_din,
    input   wire    [DATA_WIDTH/8 - 1:0]    fifo_wr_strb,
    input   wire                            fifo_wr_ena,
    output  wire                            fifo_wr_full,

    input   wire                            fifo_rd_clk,
    input   wire                            fifo_rd_nrst,
    output  wire    [15:0]                  fifo_rd_dout,
    output  wire    [1:0]                   fifo_rd_strb,
    input   wire                            fifo_rd_ena,
    output  wire                            fifo_rd_empty
);
  // FIFO contains 32-bit data word and 4-bit strobes
  localparam int unsigned FIFOWidth = DATA_WIDTH + 4;

  logic [FIFOWidth-1:0] fifo_wdata, fifo_rdata;
  logic fifo_wready, fifo_rvalid, fifo_rready;
  logic fifo_rdata_half_sel;

  assign fifo_wr_full  = ~fifo_wready;
  assign fifo_rd_empty = ~fifo_rvalid;
  assign fifo_wdata = {fifo_wr_strb, fifo_wr_din};

  prim_fifo_async #(
    .Width(FIFOWidth),
    .Depth(4)
  ) u_fifo (
    .clk_wr_i(fifo_wr_clk),
    .rst_wr_ni(fifo_wr_nrst),
    .wvalid_i(fifo_wr_ena),
    .wready_o(fifo_wready),
    .wdata_i(fifo_wdata),
    .wdepth_o(),

    .clk_rd_i(fifo_rd_clk),
    .rst_rd_ni(fifo_rd_nrst),
    .rvalid_o(fifo_rvalid),
    .rready_i(fifo_rready),
    .rdata_o(fifo_rdata),
    .rdepth_o()
  );

  assign fifo_rd_dout = fifo_rdata_half_sel ? fifo_rdata[31:16] : fifo_rdata[15:0];
  assign fifo_rd_strb = fifo_rdata_half_sel ? fifo_rdata[35:34] : fifo_rdata[33:32];
  assign fifo_rready = fifo_rd_ena & fifo_rdata_half_sel;

  always @(posedge fifo_rd_clk or negedge fifo_rd_nrst) begin
    if (!fifo_rd_nrst) begin
      fifo_rdata_half_sel <= 1'b0;
    end else if(fifo_rd_ena) begin
      fifo_rdata_half_sel <= ~fifo_rdata_half_sel;
    end
  end

  initial begin
    if (DATA_WIDTH != 32) begin
      $fatal("hbmc_dfifo only supports DATA_WIDTH of 32");
    end
  end
endmodule
