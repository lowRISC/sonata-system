// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module spi import spi_reg_pkg::*; #(
  parameter int unsigned RxFifoDepth = 64,
  parameter int unsigned TxFifoDepth = 64
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  output logic intr_rx_full_o,
  output logic intr_rx_watermark_o,
  output logic intr_tx_empty_o,
  output logic intr_tx_watermark_o,
  output logic intr_complete_o,

  output logic spi_copi_o,
  input  logic spi_cipo_i,
  output logic spi_clk_o
);

  spi_reg2hw_t reg2hw;
  spi_hw2reg_t hw2reg;

  logic unused_reg2hw;

  assign unused_reg2hw = |reg2hw;

  spi_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .intg_err_o ()
  );

  logic [7:0]  spi_data_in, spi_data_out;
  logic [10:0] spi_byte_count;
  logic        spi_data_in_valid, spi_data_in_ready;
  logic        spi_data_out_valid, spi_data_out_ready;
  logic        spi_start, spi_idle;

  logic [15:0] spi_half_clk_period;
  logic        spi_cpol, spi_cpha, spi_msb_first;

  localparam int unsigned RxFifoDepthW = prim_util_pkg::vbits(RxFifoDepth+1);

  logic [RxFifoDepthW-1:0] rx_fifo_depth;
  logic [7:0]              rx_fifo_rdata;
  logic                    rx_fifo_wvalid, rx_fifo_wready;
  logic                    rx_fifo_rready;
  logic                    rx_fifo_clr;

  localparam int unsigned TxFifoDepthW = prim_util_pkg::vbits(RxFifoDepth+1);

  logic [TxFifoDepthW-1:0] tx_fifo_depth;
  logic [7:0]              tx_fifo_wdata;
  logic                    tx_fifo_wvalid;
  logic                    tx_fifo_rvalid, tx_fifo_rready;
  logic                    tx_fifo_clr, tx_fifo_full;

  assign spi_cpol            = reg2hw.cfg.cpol.q;
  assign spi_cpha            = reg2hw.cfg.cpha.q;
  assign spi_half_clk_period = reg2hw.cfg.half_clk_period.q;
  assign spi_msb_first       = reg2hw.cfg.msb_first.q;

  assign spi_start      = (reg2hw.start.qe & spi_idle);
  assign spi_byte_count = reg2hw.start.q;

  assign tx_fifo_wvalid = reg2hw.tx_fifo.qe;
  assign tx_fifo_wdata = reg2hw.tx_fifo.q;

  assign rx_fifo_rready = reg2hw.rx_fifo.re;
  assign hw2reg.rx_fifo.d = rx_fifo_rdata;

  assign rx_fifo_clr = reg2hw.control.rx_clear.qe & reg2hw.control.rx_clear.q;
  assign tx_fifo_clr = reg2hw.control.tx_clear.qe & reg2hw.control.tx_clear.q;

  prim_fifo_sync #(
    .Width(8),
    .Pass (1'b0),
    .Depth(RxFifoDepth)
  ) u_rx_fifo (
    .clk_i,
    .rst_ni,

    .clr_i(rx_fifo_clr),

    .wvalid_i(rx_fifo_wvalid),
    .wready_o(rx_fifo_wready),
    .wdata_i (spi_data_out),

    .rvalid_o(),
    .rready_i(rx_fifo_rready),
    .rdata_o (rx_fifo_rdata),

    .full_o (),
    .depth_o(rx_fifo_depth),
    .err_o  ()
  );

  prim_fifo_sync #(
    .Width(8),
    .Pass (1'b0),
    .Depth(TxFifoDepth)
  ) u_tx_fifo (
    .clk_i,
    .rst_ni,

    .clr_i(tx_fifo_clr),

    .wvalid_i(tx_fifo_wvalid),
    .wready_o(),
    .wdata_i (tx_fifo_wdata),

    .rvalid_o(tx_fifo_rvalid),
    .rready_i(tx_fifo_rready),
    .rdata_o (spi_data_in),

    .full_o (tx_fifo_full),
    .depth_o(tx_fifo_depth),
    .err_o  ()
  );

  assign spi_data_in_valid = reg2hw.control.tx_enable.q ? tx_fifo_rvalid    : 1'b1;
  assign tx_fifo_rready    = reg2hw.control.tx_enable.q ? spi_data_in_ready : 1'b0;

  assign rx_fifo_wvalid     = reg2hw.control.rx_enable.q ? spi_data_out_valid : 1'b0;
  assign spi_data_out_ready = reg2hw.control.rx_enable.q ? rx_fifo_wready     : 1'b1;

  assign hw2reg.status.tx_fifo_level.d = {{(8 - TxFifoDepthW){1'b0}}, tx_fifo_depth};
  assign hw2reg.status.rx_fifo_level.d = {{(8 - RxFifoDepthW){1'b0}}, rx_fifo_depth};
  assign hw2reg.status.tx_fifo_full.d  = tx_fifo_full;
  assign hw2reg.status.rx_fifo_empty.d = rx_fifo_depth == '0;
  assign hw2reg.status.idle.d          = spi_idle;

  spi_core u_spi_core (
    .clk_i,
    .rst_ni,

    .data_in_i      (spi_data_in),
    .data_in_valid_i(spi_data_in_valid),
    .data_in_ready_o(spi_data_in_ready),

    .data_out_o      (spi_data_out),
    .data_out_valid_o(spi_data_out_valid),
    .data_out_ready_i(spi_data_out_ready),

    .start_i     (spi_start),
    .byte_count_i(spi_byte_count),
    .idle_o      (spi_idle),

    .cpol_i           (spi_cpol),
    .cpha_i           (spi_cpha),
    .msb_first_i      (spi_msb_first),
    .half_clk_period_i(spi_half_clk_period),

    .spi_copi_o,
    .spi_cipo_i,
    .spi_clk_o
  );

  assign hw2reg.intr_state = '0;

  assign intr_rx_full_o      = 1'b0;
  assign intr_rx_watermark_o = 1'b0;
  assign intr_tx_empty_o     = 1'b0;
  assign intr_tx_watermark_o = 1'b0;
  assign intr_complete_o     = 1'b0;
endmodule
