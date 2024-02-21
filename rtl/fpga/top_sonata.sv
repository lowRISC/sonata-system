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

  output logic ser0_tx,
  input  logic ser0_rx
);
  parameter SRAMInitFile = "";

  logic top_rst_n;
  logic main_clk_buf;
  logic clk_sys, rst_sys_n;
  logic [7:0] reset_counter;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;

  initial begin
    reset_counter = 0;
  end

  always_ff @(posedge main_clk_buf) begin
    if (reset_counter != 8'hff) begin
      reset_counter <= reset_counter + 8'd1;
    end
  end

  assign top_rst_n = reset_counter < 8'd5   ? 1'b1 :
                     reset_counter < 8'd200 ? 1'b0 :
                                              nrst ;

  assign led_bootok = 1'b1;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~navSw;
  assign user_sw_n = ~usrSw;

  // No LCD backlight FPGA IO on v0.2 board, so leave this unconnected.
  logic lcd_backlight;

  sonata_system #(
    .GpiWidth    ( 13           ),
    .GpoWidth    ( 24           ),
    .PwmWidth    ( 0            ),
    .SRAMInitFile( SRAMInitFile )
  ) u_sonata_system (
    .clk_sys_i (clk_sys),
    .rst_sys_ni(rst_sys_n),

    .gp_i({user_sw_n, nav_sw_n}),
    .gp_o({cheriErr, led_legacy, led_cheri, led_halted, usrLed, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i(ser0_rx),
    .uart_tx_o(ser0_tx),

    .pwm_o(),

    .spi_rx_i (1'b0),
    .spi_tx_o (lcd_copi),
    .spi_sck_o(lcd_clk)
  );

  // Produce 50 MHz system clock from 25 MHz Sonata board clock.
  clkgen_sonata clkgen(
    .IO_CLK    (mainClk),
    .IO_CLK_BUF(main_clk_buf),
    .IO_RST_N  (top_rst_n),
    .clk_sys,
    .rst_sys_n
  );

endmodule
