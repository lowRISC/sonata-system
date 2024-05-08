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

  output logic       lcd_rst,
  output logic       lcd_dc,
  output logic       lcd_copi,
  output logic       lcd_clk,
  output logic       lcd_cs,
  output logic       lcd_backlight,

  output logic       ethmac_rst,
  output logic       ethmac_copi,
  output logic       ethmac_sclk,
  input  logic       ethmac_cipo,
  input  logic       ethmac_intr,
  output logic       ethmac_cs,

  output logic       rgbled0,

  output logic       ser0_tx,
  input  logic       ser0_rx,

  // I2C buses
  inout  logic       scl0,
  inout  logic       sda0,

  output logic       scl1,
  output logic       sda1,

  // Status input from USB transceiver
  input  logic       usrusb_vbusdetect,

  // Control of USB transceiver
  output logic       usrusb_softcn,
  // Configure the USB transceiver for Full Speed operation.
  output logic       usrusb_spd,

  // Reception from USB host via transceiver
  input  logic       usrusb_v_p,
  input  logic       usrusb_v_n,
  input  logic       usrusb_rcv,

  // Transmission to USB host via transceiver
  output logic       usrusb_vpo,
  output logic       usrusb_vmo,

  // Always driven configuration signals to the USB transceiver.
  output logic       usrusb_oe,
  output logic       usrusb_sus,

  input  logic       tck_i,
  input  logic       tms_i,
  input  logic       td_i,
  output logic       td_o,

  output logic appspi_clk,
  output logic appspi_d0, // COPI (controller output peripheral input)
  input  logic appspi_d1, // CIPO (controller input peripheral output)
  output logic appspi_d2, // WP_N (write protect negated)
  output logic appspi_d3, // HOLD_N or RESET_N
  output logic appspi_cs  // Chip select negated
);
  // System clock frequency.
  parameter int SysClkFreq = 25_000_000;

  parameter SRAMInitFile = "";

  // Main system clock and reset
  logic main_clk_buf;
  logic clk_sys;
  logic rst_sys_n;

  // USB device clock and reset
  logic clk_usb;
  wire  rst_usb_n = rst_sys_n;

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

  assign usrusb_spd = 1'b1;  // Full Speed operation.

  logic dp_en_d2p;
  logic rx_enable_d2p;
  assign usrusb_oe  = !dp_en_d2p;  // Active low Output Enable.
  assign usrusb_sus = !rx_enable_d2p;

  logic cheri_en;

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;

  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Open Drain drivers onto I2C bus.
  assign scl0 = scl0_oe ? scl0_o : 1'bZ;
  assign sda0 = sda0_oe ? sda0_o : 1'bZ;

  assign scl1 = scl1_oe ? scl1_o : 1'bZ;
  assign sda1 = sda1_oe ? sda1_o : 1'bZ;

  // DIP switches are OFF by default and employ a pull up, which means CHERI shall enabled by
  // default; sample this once when leaving reset as a development aid during bring up.
  logic enable_cheri;
  logic cheri_decided;
  always_ff @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      cheri_decided <= 1'b0;
      enable_cheri  <= 1'b1;
    end else if (!cheri_decided) begin
      enable_cheri  <= usrSw[7];
      cheri_decided <= 1'b1;
    end
  end

  sonata_system #(
    .GpiWidth     ( 13           ),
    .GpoWidth     ( 15           ),
    .PwmWidth     (  0           ),
    .CheriErrWidth(  9           ),
    .SRAMInitFile ( SRAMInitFile )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_sys),
    .rst_sys_ni     (rst_sys_n),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    .gp_i           ({user_sw_n, nav_sw_n}),
    .gp_o           ({ethmac_rst, ethmac_cs, appspi_cs, usrLed, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i      (ser0_rx),
    .uart_tx_o      (ser0_tx),

    .pwm_o(),

    .spi_lcd_rx_i   (1'b0),
    .spi_lcd_tx_o   (lcd_copi),
    .spi_lcd_sck_o  (lcd_clk),

    .spi_flash_rx_i (appspi_d1),
    .spi_flash_tx_o (appspi_d0),
    .spi_flash_sck_o(appspi_clk),

    .spi_eth_rx_i   (ethmac_cipo),
    .spi_eth_tx_o   (ethmac_copi),
    .spi_eth_sck_o  (ethmac_sclk),
    .spi_eth_irq_ni (ethmac_intr),

    .cheri_en_i     (enable_cheri),
    .cheri_err_o    (cheriErr),
    .cheri_en_o     (cheri_en),

    // I2C bus 0
    .i2c0_scl_i     (scl0),
    .i2c0_scl_o     (scl0_o),
    .i2c0_scl_en_o  (scl0_oe),
    .i2c0_sda_i     (sda0),
    .i2c0_sda_o     (sda0_o),
    .i2c0_sda_en_o  (sda0_oe),

    // I2C bus 1
    .i2c1_scl_i     (scl1),
    .i2c1_scl_o     (scl1_o),
    .i2c1_scl_en_o  (scl1_oe),
    .i2c1_sda_i     (sda1),
    .i2c1_sda_o     (sda1_o),
    .i2c1_sda_en_o  (sda1_oe),

    // Reception from USB host via transceiver
    .usb_dp_i         (usrusb_v_p),
    .usb_dn_i         (usrusb_v_n),
    .usb_rx_d_i       (usrusb_rcv),

    // Transmission to USB host via transceiver
    .usb_dp_o         (usrusb_vpo),
    .usb_dp_en_o      (dp_en_d2p),
    .usb_dn_o         (usrusb_vmo),
    .usb_dn_en_o      (),

    // Configuration and control of USB transceiver
    .usb_sense_i      (usrusb_vbusdetect),
    .usb_dp_pullup_o  (usrusb_softcn),
    .usb_dn_pullup_o  (),
    .usb_rx_enable_o  (rx_enable_d2p),

    .tck_i,
    .tms_i,
    .trst_ni(rst_sys_n),
    .td_i,
    .td_o
  );

  // Tie flash wp_n and hold_n to 1 as they're active low and we don't need either signal
  assign appspi_d2 = 1'b1;
  assign appspi_d3 = 1'b1;

  assign led_cheri = cheri_en;
  assign led_legacy = ~cheri_en;
  assign led_halted = 1'b0;

  // Produce 50 MHz system clock from 25 MHz Sonata board clock.
  clkgen_sonata #(
    .SysClkFreq(SysClkFreq)
  ) u_clkgen(
    .IO_CLK    (mainClk),
    .IO_CLK_BUF(main_clk_buf),
    .clk_sys,
    .clk_usb,
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
