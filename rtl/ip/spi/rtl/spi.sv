// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module spi import spi_reg_pkg::*; #(
  parameter int unsigned RxFifoDepth = 8,
  parameter int unsigned TxFifoDepth = 8,
  // Maximum number of peripherals supported by this controller.
  parameter int unsigned CSWidth = spi_reg_pkg::MaxPeripherals
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Interrupt lines.
  output logic intr_rx_full_o,
  output logic intr_rx_watermark_o,
  output logic intr_tx_empty_o,
  output logic intr_tx_watermark_o,
  output logic intr_complete_o,

  // SPI bus signals.
  output logic               spi_copi_o,
  input  logic               spi_cipo_i,
  output logic [CSWidth-1:0] spi_cs_o,
  output logic               spi_clk_o
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
    .hw2reg
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
  logic                    rx_fifo_clr, rx_fifo_full;

  localparam int unsigned TxFifoDepthW = prim_util_pkg::vbits(TxFifoDepth+1);

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

  // Report the FIFO properties.
  assign hw2reg.info.rx_fifo_depth.d = 8'(RxFifoDepth);
  assign hw2reg.info.tx_fifo_depth.d = 8'(TxFifoDepth);

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

    .full_o (rx_fifo_full),
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

  // Event-type completion interrupt; generate an interrupt when `spi_idle` is first asserted.
  logic event_complete;
  prim_edge_detector #(
    .Width(1),
    .EnSync(1'b0)
  ) u_gen_event_complete (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .d_i                (spi_idle),
    .q_sync_o           (),
    .q_posedge_pulse_o  (event_complete),
    .q_negedge_pulse_o  ()
  );

  // Widen the FIFO levels to 8 bits for the following comparisons and for presentation in the
  // registers.
  logic [7:0] rx_fifo_depth_w, tx_fifo_depth_w;
  assign rx_fifo_depth_w = 8'(rx_fifo_depth);
  assign tx_fifo_depth_w = 8'(tx_fifo_depth);

  // Status-type FIFO-related interrupts.
  logic rx_fifo_ge_watermark, tx_fifo_empty, tx_fifo_le_watermark;
  assign tx_fifo_empty = ~|tx_fifo_depth;
  // Tx FIFO level at or above programmed watermark (1,2,4,8,16,32,56)
  always_comb begin
    if (reg2hw.control.rx_watermark.q == 4'h6) rx_fifo_ge_watermark = (rx_fifo_depth_w >= 8'd56);
    else rx_fifo_ge_watermark = |(rx_fifo_depth_w >> reg2hw.control.rx_watermark.q);
  end
  // Rx FIFO level at or below programmed watermark (1,2,4,8,16)
  assign tx_fifo_le_watermark = (tx_fifo_depth_w == (8'h1 << reg2hw.control.tx_watermark.q)) ||
                              ~|(tx_fifo_depth_w >> reg2hw.control.tx_watermark.q);

  assign spi_data_in_valid = reg2hw.control.tx_enable.q ? tx_fifo_rvalid    : 1'b1;
  assign tx_fifo_rready    = reg2hw.control.tx_enable.q ? spi_data_in_ready : 1'b0;

  assign rx_fifo_wvalid     = reg2hw.control.rx_enable.q ? spi_data_out_valid : 1'b0;
  assign spi_data_out_ready = reg2hw.control.rx_enable.q ? rx_fifo_wready     : 1'b1;

  assign hw2reg.status.tx_fifo_level.d = tx_fifo_depth_w;
  assign hw2reg.status.rx_fifo_level.d = rx_fifo_depth_w;
  assign hw2reg.status.tx_fifo_full.d  = tx_fifo_full;
  assign hw2reg.status.rx_fifo_empty.d = ~|rx_fifo_depth;
  assign hw2reg.status.idle.d          = spi_idle;

  // Software reset of the core logic.
  logic sw_reset;
  assign sw_reset = reg2hw.control.sw_reset.qe & reg2hw.control.sw_reset.q;

  // Internal loopback functionality allowing the input (CIPO) to be received directly from
  // the output (COPI) for testing.
  logic spi_cipo;
  assign spi_cipo = reg2hw.control.int_loopback.q ? spi_copi_o : spi_cipo_i;

  spi_core u_spi_core (
    .clk_i,
    .rst_ni,

    .sw_reset_i       (sw_reset),

    .data_in_i        (spi_data_in),
    .data_in_valid_i  (spi_data_in_valid),
    .data_in_ready_o  (spi_data_in_ready),

    .data_out_o       (spi_data_out),
    .data_out_valid_o (spi_data_out_valid),
    .data_out_ready_i (spi_data_out_ready),

    .start_i          (spi_start),
    .byte_count_i     (spi_byte_count),
    .idle_o           (spi_idle),

    .cpol_i           (spi_cpol),
    .cpha_i           (spi_cpha),
    .msb_first_i      (spi_msb_first),
    .half_clk_period_i(spi_half_clk_period),

    .spi_copi_o,
    .spi_cipo_i       (spi_cipo),
    .spi_clk_o
  );

  // CS signals come directly from the register interface and shall be modified only when the
  // core is Idle.
  always_comb begin : spi_cs_output
    for (int unsigned p = 0; p < CSWidth; p++) begin
      spi_cs_o[p] = reg2hw.cs[p].q;
    end
  end

  // Interrupts
  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_rx_full (
    .clk_i,
    .rst_ni,
    .event_intr_i           (rx_fifo_full),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_full.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_full.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_full.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_full.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_full.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_full.d),
    .intr_o                 (intr_rx_full_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_rx_watermark (
    .clk_i,
    .rst_ni,
    .event_intr_i           (rx_fifo_ge_watermark),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.rx_watermark.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.rx_watermark.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.rx_watermark.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.rx_watermark.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.rx_watermark.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.rx_watermark.d),
    .intr_o                 (intr_rx_watermark_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_tx_empty (
    .clk_i,
    .rst_ni,
    .event_intr_i           (tx_fifo_empty),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.tx_empty.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.tx_empty.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.tx_empty.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.tx_empty.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.tx_empty.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.tx_empty.d),
    .intr_o                 (intr_tx_empty_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Status")) intr_tx_watermark (
    .clk_i,
    .rst_ni,
    .event_intr_i           (tx_fifo_le_watermark),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.tx_watermark.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.tx_watermark.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.tx_watermark.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.tx_watermark.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.tx_watermark.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.tx_watermark.d),
    .intr_o                 (intr_tx_watermark_o)
  );

  prim_intr_hw #(.Width(1), .IntrT("Event")) intr_complete (
    .clk_i,
    .rst_ni,
    .event_intr_i           (event_complete),
    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.complete.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.complete.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.complete.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.complete.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.complete.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.complete.d),
    .intr_o                 (intr_complete_o)
  );
endmodule
