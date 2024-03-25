// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Sonata system top level for the Sonata PCB
module top_sonata (
  input  logic mainClk,
  input  logic nrst,

  output logic [7:0] usrLed,
  output logic       led_bootok,
  output logic       led_halted,
  output logic       led_cheri,
  output logic       led_legacy,
  output logic [8:0] cheriErr,

  input  logic [4:0] navSw,
  input  logic [7:0] usrSw,

  output logic lcd_rst,
  output logic lcd_dc,
  output logic lcd_copi,
  output logic lcd_clk,
  output logic lcd_cs,
  output logic lcd_backlight,

  output logic rgbled0,

  output logic ser0_tx,
  input  logic ser0_rx
);
  parameter SRAMInitFile = "";

  logic main_clk_buf;
  logic clk_sys;
  logic rst_sys_n;
  logic [7:0] reset_counter;
  logic pll_locked;
  logic rst_btn;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;

  assign led_bootok = rst_sys_n;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~navSw;
  assign user_sw_n = ~usrSw;

  logic cheri_en;

  sonata_system #(
    .GpiWidth     ( 13           ),
    .GpoWidth     ( 12           ),
    .PwmWidth     (  0           ),
    .CheriErrWidth(  9           ),
    .SRAMInitFile ( SRAMInitFile )
  ) u_sonata_system (
    .clk_sys_i (clk_sys),
    .rst_sys_ni(rst_sys_n),

    .gp_i({user_sw_n, nav_sw_n}),
    .gp_o({usrLed, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i(ser0_rx),
    .uart_tx_o(ser0_tx),

    .pwm_o(),

    .spi_rx_i (1'b0),
    .spi_tx_o (lcd_copi),
    .spi_sck_o(lcd_clk),

    .cheri_err_o(cheriErr),
    .cheri_en_o (cheri_en)
  );

  assign led_cheri = cheri_en;
  assign led_legacy = ~cheri_en;
  assign led_halted = 1'b0;

  // Produce 50 MHz system clock from 25 MHz Sonata board clock.
  clkgen_sonata u_clkgen(
    .IO_CLK    (mainClk),
    .IO_CLK_BUF(main_clk_buf),
    .clk_sys,
    .locked    (pll_locked)
  );

  // Produce reset signal at beginning of time and when button pressed.
  assign rst_btn = ~nrst;

  rst_ctrl u_rst_ctrl (
    .clk_i       (main_clk_buf),
    .pll_locked_i(pll_locked),
    .rst_btn_i   (rst_btn),
    .rst_no      (rst_sys_n)
  );

  // Drive RGB LEDs to off state
  logic rgb_led_data_last;
  logic rgb_led_data_ack;
  logic rgb_led_data_out;

  always_ff @(posedge main_clk_buf or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      rgb_led_data_last <= 1'b0;
    end else begin
      if (rgb_led_data_ack) begin
        rgb_led_data_last <= ~rgb_led_data_last;
      end
    end
  end

  assign rgbled0 = ~rgb_led_data_out;

  ws281x_drv u_rgb_led_drv (
    .clk_i(main_clk_buf),
    .rst_ni(rst_sys_n),

    .go_i(1'b1),
    .idle_o(),
    .data_i({8'd0, 8'd0, 8'd0}),
    .data_valid_i(1'b1),
    .data_last_i(rgb_led_data_last),
    .data_ack_o(rgb_led_data_ack),
    .ws281x_dout_o(rgb_led_data_out)
  );

endmodule
