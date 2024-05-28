// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module rgbled_ctrl import rgbled_ctrl_reg_pkg::*; #(
  parameter int unsigned CycleTime = 31
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  output logic rgbled_dout_o
);

  rgbled_ctrl_reg2hw_t reg2hw;
  rgbled_ctrl_hw2reg_t hw2reg;

  logic go, drv_idle, idle, off;
  logic [23:0] grb_data;
  logic grb_data_valid, grb_data_last, grb_data_ack;
  logic grb_data_sel_q, grb_data_sel_d;

  logic [23:0] grb0_q, grb0_d, grb1_q, grb1_d;
  logic        grb0_en, grb1_en;

  logic reset_turn_off_q, reset_turn_off_d;

  assign grb0_en = idle & (reg2hw.rgbled0.b.qe | off);
  assign grb0_d  = reg2hw.rgbled0.b.qe ? {reg2hw.rgbled0.g.q, reg2hw.rgbled0.r.q, reg2hw.rgbled0.b.q} :
                                         '0;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb0_q <= '0;
    end else begin
      if (grb0_en) begin
        grb0_q <= grb0_d;
      end
    end
  end

  assign grb1_en = idle & (reg2hw.rgbled1.b.qe | off);
  assign grb1_d  = reg2hw.rgbled1.b.qe ? {reg2hw.rgbled1.g.q, reg2hw.rgbled1.r.q, reg2hw.rgbled1.b.q} :
                                         '0;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb1_q <= '0;
    end else begin
      if (grb1_en) begin
        grb1_q <= grb1_d;
      end
    end
  end

  assign grb_data_valid = 1'b1;
  assign grb_data       = off            ? '0     :
                          grb_data_sel_q ? grb0_q :
                                           grb1_q;
  assign grb_data_last  = grb_data_sel_q;

  assign grb_data_sel_d = grb_data_ack ? ~grb_data_sel_q : grb_data_sel_q;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb_data_sel_q   <= 1'b0;
      reset_turn_off_q <= 1'b1;
    end else begin
      grb_data_sel_q   <= grb_data_sel_d;
      reset_turn_off_q <= reset_turn_off_d;
    end
  end

  assign reset_turn_off_d = reset_turn_off_q ? ~drv_idle : 1'b0;

  rgbled_ctrl_reg_top u_rgbled_ctrl_reg_top (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .intg_err_o()
  );

  assign off = (reg2hw.ctrl.off.qe    & reg2hw.ctrl.off.q)    | reset_turn_off_q;
  assign go  = (reg2hw.ctrl.setrgb.qe & reg2hw.ctrl.setrgb.q) | off;

  ws281x_drv #(
    .CycleTime(CycleTime)
  ) u_ws281x_drv (
    .clk_i,
    .rst_ni,

    .go_i  (go),
    .idle_o(drv_idle),

    .data_i      (grb_data),
    .data_valid_i(grb_data_valid),
    .data_last_i (grb_data_last),
    .data_ack_o  (grb_data_ack),

    .ws281x_dout_o(rgbled_dout_o)
  );

  assign idle = drv_idle & ~reset_turn_off_q;

  assign hw2reg.status.d = idle;
endmodule
