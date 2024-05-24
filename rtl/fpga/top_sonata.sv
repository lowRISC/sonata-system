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

  // UART 0
  output logic       ser0_tx,
  input  logic       ser0_rx,

  // UART 1
  output logic       ser1_tx,
  input  logic       ser1_rx,

  // QWIIC (Sparkfun) buses
  inout  logic       scl0,  // qwiic0 and Arduino Header
  inout  logic       sda0,

  inout  logic       scl1,  // qwiic1
  inout  logic       sda1,

  // R-Pi header I2C buses
  inout  logic       rph_g3_scl,  // SCL1/GPIO3 on Header
  inout  logic       rph_g2_sda,  // SDA1/GPIO2

  inout  logic       rph_g1,  // ID_SC for HAT ID EEPROM
  inout  logic       rph_g0,  // ID_SD

  // R-Pi header SPI buses
  output logic       rph_g11_sclk, // SPI0
  output logic       rph_g10_copi, // SPI0
  input  logic       rph_g9_cipo,  // SPI0
  output logic       rph_g8_ce0,   // SPI0
  output logic       rph_g7_ce1,   // SPI0

  output logic       rph_g21_sclk, // SPI1
  output logic       rph_g20_copi, // SPI1
  input  logic       rph_g19_cipo, // SPI1
  output logic       rph_g18,      // SPI1 CE0
  output logic       rph_g17,      // SPI1 CE1
  output logic       rph_g16_ce2,  // SPI1

  // Arduino shield SPI bus
  output logic       ah_tmpio10, // Chip select
  output logic       ah_tmpio14, // CIPO
  input  logic       ah_tmpio15, // SCK
  output logic       ah_tmpio17, // COPI

  // mikroBUS Click I2C bus
  inout  logic       mb6,     // SCL
  inout  logic       mb5,     // SDA

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

  // User JTAG
  input  logic       tck_i,
  input  logic       tms_i,
  input  logic       td_i,
  output logic       td_o,

  // SPI flash interface
  output logic       appspi_clk,
  output logic       appspi_d0, // COPI (controller output peripheral input)
  input  logic       appspi_d1, // CIPO (controller input peripheral output)
  output logic       appspi_d2, // WP_N (write protect negated)
  output logic       appspi_d3, // HOLD_N or RESET_N
  output logic       appspi_cs  // Chip select negated
);
  // System clock frequency.
  parameter int SysClkFreq = 30_000_000;

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

  // Open Drain drivers onto I2C buses.
  // TODO: move this into two parameterised I2C splitter modules?
  assign scl0 = scl0_oe ? scl0_o : 1'bZ;
  assign sda0 = sda0_oe ? sda0_o : 1'bZ;

  assign scl1 = scl1_oe ? scl1_o : 1'bZ;
  assign sda1 = sda1_oe ? sda1_o : 1'bZ;

  // I2C bus to GPIO2/3
  assign rph_g3_scl = scl1_oe ? scl1_o : 1'bZ;
  assign rph_g2_sda = sda1_oe ? sda1_o : 1'bZ;

  // HAT ID EEPROM
  assign rph_g1 = scl0_oe ? scl0_o : 1'bZ;
  assign rph_g0 = sda0_oe ? sda0_o : 1'bZ;

  // mikroBUS Click I2C bus
  assign mb6 = scl1_oe ? scl1_o : 1'bZ;
  assign mb5 = sda1_oe ? sda1_o : 1'bZ;

  // Inputs from I2C buses.
  wire scl0_i = scl0 & rph_g1;
  wire sda0_i = sda0 & rph_g0;

  wire scl1_i = scl1 & rph_g3_scl & mb6;
  wire sda1_i = sda1 & rph_g2_sda & mb5;

  // Enable CHERI by default.
  logic enable_cheri;
  assign enable_cheri = 1'b1;

  sonata_system #(
    .GpiWidth     ( 13           ),
    .GpoWidth     ( 21           ),
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
    .gp_o           ({
                      ah_tmpio10, // Arduino shield chip select
                      rph_g18, rph_g17, rph_g16_ce2, // R-Pi SPI1 chip select
                      rph_g8_ce0, rph_g7_ce1, // R-Pi SPI0 chip select
                      ethmac_rst, ethmac_cs, // Ethernet
                      appspi_cs, // Flash
                      usrLed, // User LEDs (8 bits)
                      lcd_backlight, lcd_dc, lcd_rst, lcd_cs // LCD screen
                    }),

    // UART 0
    .uart0_rx_i     (ser0_rx),
    .uart0_tx_o     (ser0_tx),

    // UART 1
    .uart1_rx_i     (ser1_rx),
    .uart1_tx_o     (ser1_tx),

    .pwm_o(),

    // SPI for LCD screen
    .spi_lcd_rx_i   (1'b0),
    .spi_lcd_tx_o   (lcd_copi),
    .spi_lcd_sck_o  (lcd_clk),

    // SPI for flash memory
    .spi_flash_rx_i (appspi_d1),
    .spi_flash_tx_o (appspi_d0),
    .spi_flash_sck_o(appspi_clk),

    // SPI for ethernet
    .spi_eth_rx_i   (ethmac_cipo),
    .spi_eth_tx_o   (ethmac_copi),
    .spi_eth_sck_o  (ethmac_sclk),
    .spi_eth_irq_ni (ethmac_intr),

    // SPI0 on the R-Pi header
    .spi_rp0_rx_i   (rph_g9_cipo),
    .spi_rp0_tx_o   (rph_g10_copi),
    .spi_rp0_sck_o  (rph_g11_sclk),

    // SPI1 on the R-Pi header
    .spi_rp1_rx_i   (rph_g19_cipo),
    .spi_rp1_tx_o   (rph_g20_copi),
    .spi_rp1_sck_o  (rph_g21_sclk),

    // SPI on Arduino shield
    .spi_ard_rx_i   (ah_tmpio14), // CIPO
    .spi_ard_tx_o   (ah_tmpio17), // COPI
    .spi_ard_sck_o  (ah_tmpio15), // SCLK

    // CHERI signals
    .cheri_en_i     (enable_cheri),
    .cheri_err_o    (cheriErr),
    .cheri_en_o     (cheri_en),

    // I2C bus 0
    .i2c0_scl_i     (scl0_i),
    .i2c0_scl_o     (scl0_o),
    .i2c0_scl_en_o  (scl0_oe),
    .i2c0_sda_i     (sda0_i),
    .i2c0_sda_o     (sda0_o),
    .i2c0_sda_en_o  (sda0_oe),

    // I2C bus 1
    .i2c1_scl_i     (scl1_i),
    .i2c1_scl_o     (scl1_o),
    .i2c1_scl_en_o  (scl1_oe),
    .i2c1_sda_i     (sda1_i),
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

    // User JTAG
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
