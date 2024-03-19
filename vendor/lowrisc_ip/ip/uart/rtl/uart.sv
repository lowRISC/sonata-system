// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Description: UART top level wrapper file

`include "prim_assert.sv"

module uart import uart_reg_pkg::*; (
  input           clk_i,
  input           rst_ni,

  // Bus Interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Generic IO
  input           cio_rx_i,
  output logic    cio_tx_o,
  output logic    cio_tx_en_o,

  // Interrupts
  output logic    intr_tx_watermark_o ,
  output logic    intr_rx_watermark_o ,
  output logic    intr_tx_empty_o  ,
  output logic    intr_rx_overflow_o  ,
  output logic    intr_rx_frame_err_o ,
  output logic    intr_rx_break_err_o ,
  output logic    intr_rx_timeout_o   ,
  output logic    intr_rx_parity_err_o
);

  uart_reg2hw_t reg2hw;
  uart_hw2reg_t hw2reg;

  uart_reg_top u_reg (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg
  );

  uart_core uart_core (
    .clk_i,
    .rst_ni,
    .reg2hw,
    .hw2reg,

    .rx    (cio_rx_i   ),
    .tx    (cio_tx_o   ),

    .intr_tx_watermark_o,
    .intr_rx_watermark_o,
    .intr_tx_empty_o,
    .intr_rx_overflow_o,
    .intr_rx_frame_err_o,
    .intr_rx_break_err_o,
    .intr_rx_timeout_o,
    .intr_rx_parity_err_o
  );

  // always enable the driving out of TX
  assign cio_tx_en_o = 1'b1;

  // Assert Known for outputs
  `ASSERT(TxEnIsOne_A, cio_tx_en_o === 1'b1)
  `ASSERT_KNOWN(TxKnown_A, cio_tx_o, clk_i, !rst_ni || !cio_tx_en_o)

  // Assert Known for interrupts
  `ASSERT_KNOWN(TxWatermarkKnown_A, intr_tx_watermark_o)
  `ASSERT_KNOWN(RxWatermarkKnown_A, intr_rx_watermark_o)
  `ASSERT_KNOWN(TxEmptyKnown_A, intr_tx_empty_o)
  `ASSERT_KNOWN(RxOverflowKnown_A, intr_rx_overflow_o)
  `ASSERT_KNOWN(RxFrameErrKnown_A, intr_rx_frame_err_o)
  `ASSERT_KNOWN(RxBreakErrKnown_A, intr_rx_break_err_o)
  `ASSERT_KNOWN(RxTimeoutKnown_A, intr_rx_timeout_o)
  `ASSERT_KNOWN(RxParityErrKnown_A, intr_rx_parity_err_o)
endmodule
