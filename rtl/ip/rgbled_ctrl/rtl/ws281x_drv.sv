// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module ws281x_drv #(
  parameter int unsigned CycleTime = 31
) (
  input clk_i,
  input rst_ni,

  input              go_i,
  output             idle_o,

  input logic [23:0] data_i,
  input logic        data_valid_i,
  input logic        data_last_i,
  output logic       data_ack_o,

  output logic       ws281x_dout_o
);
  localparam int unsigned ResetClkCnt = CycleTime * 250;
  localparam int unsigned Tx0HClkCnt = (CycleTime * 3) / 10;
  localparam int unsigned Tx1HClkCnt = (CycleTime * 7) / 10;
  localparam int unsigned ClkCounterW = $clog2(ResetClkCnt);

  typedef enum logic [2:0] {
    IDLE       = 3'b000,
    TX_0_H     = 3'b101,
    TX_1_H     = 3'b111,
    TX_L       = 3'b100,
    TX_L_END   = 3'b110,
    RESET_WAIT = 3'b010
  } state_e;

  state_e state_q, state_d;

  logic [22:0]            cur_data_q, cur_data_d;
  logic                   cur_data_last_q, cur_data_last_d;
  logic [4:0]             bit_cnt_q, bit_cnt_d;
  logic [ClkCounterW-1:0] h_end_clk_cnt;
  logic [ClkCounterW-1:0] clk_cnt_q, clk_cnt_d;
  logic                   reset_clk_cnt;

  always_comb begin
    state_d         = state_q;
    cur_data_d      = cur_data_q;
    cur_data_last_d = cur_data_last_q;
    reset_clk_cnt   = 1'b0;
    data_ack_o      = 1'b0;
    bit_cnt_d       = bit_cnt_q;

    case(state_q)
      IDLE: begin
        reset_clk_cnt = 1'b1;

        if(go_i) begin
          cur_data_d      = data_i[22:0];
          cur_data_last_d = data_last_i;
          data_ack_o      = 1'b1;
          state_d         = data_i[23] ? TX_1_H : TX_0_H;
          reset_clk_cnt   = 1'b1;
          bit_cnt_d       = 5'd23;
        end
      end
      TX_0_H, TX_1_H: begin
        if (clk_cnt_q == h_end_clk_cnt) begin
          state_d = TX_L;
        end
      end
      TX_L: begin
        if (clk_cnt_q == ClkCounterW'(CycleTime)) begin
          if (bit_cnt_q == 0) begin
            reset_clk_cnt = 1'b1;

            if (cur_data_last_q) begin
              state_d = RESET_WAIT;
            end else begin
              state_d = TX_L_END;
            end
          end else begin
            state_d       = cur_data_q[22] ? TX_1_H : TX_0_H;
            reset_clk_cnt = 1'b1;
            cur_data_d    = {cur_data_q[21:0], 1'b0};
            bit_cnt_d     = bit_cnt_q - 1'b1;
          end
        end
      end
      TX_L_END: begin
        reset_clk_cnt = 1'b1;

        if(data_valid_i) begin
          cur_data_d    = data_i[22:0];
          cur_data_last_d = data_last_i;
          data_ack_o    = 1'b1;
          state_d       = data_i[23] ? TX_1_H : TX_0_H;
          bit_cnt_d     = 5'd23;
        end
      end
      RESET_WAIT: begin
        if (clk_cnt_q == ClkCounterW'(ResetClkCnt)) begin
          state_d    = IDLE;
        end
      end
      default: begin
      end
    endcase
  end

  assign h_end_clk_cnt = ClkCounterW'(state_q == TX_0_H ? Tx0HClkCnt : Tx1HClkCnt);
  assign clk_cnt_d = reset_clk_cnt ? '0 : clk_cnt_q + 1'b1;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      clk_cnt_q <= '0;
    end else begin
      clk_cnt_q <= clk_cnt_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= IDLE;
    end else begin
      state_q <= state_d;
    end
  end

  always_ff @(posedge clk_i) begin
    cur_data_last_q <= cur_data_last_d;
    cur_data_q      <= cur_data_d;
    bit_cnt_q       <= bit_cnt_d;
  end

  assign idle_o        = state_q == IDLE;
  assign ws281x_dout_o = state_q[0];
endmodule
