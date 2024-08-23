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

  // RS-232
  output logic       rs232_tx,
  input  logic       rs232_rx,

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

  // R-Pi header UART
  output logic       rph_txd0,
  input  logic       rph_rxd0,

  // R-Pi header GPIO
  inout  logic       rph_g27,
  inout  logic       rph_g26,
  inout  logic       rph_g25,
  inout  logic       rph_g24,
  inout  logic       rph_g23,
  inout  logic       rph_g22,
  inout  logic       rph_g13,
  inout  logic       rph_g12,
  inout  logic       rph_g6,
  inout  logic       rph_g5,
  inout  logic       rph_g4,

  // Arduino shield GPIO
  inout  logic       ah_tmpio0,
  inout  logic       ah_tmpio1,
  inout  logic       ah_tmpio2,
  inout  logic       ah_tmpio3,
  inout  logic       ah_tmpio4,
  inout  logic       ah_tmpio5,
  inout  logic       ah_tmpio6,
  inout  logic       ah_tmpio7,
  inout  logic       ah_tmpio8,
  inout  logic       ah_tmpio9,
  inout  logic       ah_tmpio16,

  // Arduino shield SPI bus
  output logic       ah_tmpio10, // Chip select
  output logic       ah_tmpio11, // COPI
  input  logic       ah_tmpio12, // CIPO
  output logic       ah_tmpio13, // SCLK

  // Arduino shield analog(ue) pins digital inputs
  input logic [5:0]  ard_an_di,

  // Arduino shield analog(ue) pins actual analog(ue) input pairs
  input wire  [5:0]  ard_an_p,
  input wire  [5:0]  ard_an_n,

  // mikroBUS Click other
  output logic       mb10, // PWM
  input  logic       mb9,  // Interrupt
  output logic       mb0,  // Reset

  // mikroBUS Click UART
  input  logic       mb8,  // RX
  output logic       mb7,  // TX

  // mikroBUS Click I2C bus
  inout  logic       mb6,  // SCL
  inout  logic       mb5,  // SDA

  // mikroBUS Click SPI
  output logic       mb4,  // COPI
  input  logic       mb3,  // CIPO
  output logic       mb2,  // SCK
  output logic       mb1,  // Chip select

  // PMODs
  inout  logic [7:0] pmod0,
  inout  logic [7:0] pmod1,

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
  output logic       appspi_cs, // Chip select negated

  inout  wire [7:0]  hyperram_dq,
  inout  wire        hyperram_rwds,
  output wire        hyperram_ckp,
  output wire        hyperram_ckn,
  output wire        hyperram_nrst,
  output wire        hyperram_cs
);
  // System clock frequency.
  parameter int unsigned SysClkFreq = 30_000_000;
  parameter int unsigned HRClkFreq  = 100_000_000;

  parameter SRAMInitFile   = "";
  parameter EnableHyperram = 1'b1;

  // Main system clock and reset
  logic main_clk_buf;
  logic clk_sys;
  logic rst_sys_n;

  // USB device clock and reset
  logic clk_usb;
  wire  rst_usb_n = rst_sys_n;

  logic clk_hr, clk_hr90p, clk_hr3x;

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

  logic [15:0] rp_gp_oe;
  logic [15:0] rp_gp_o;

  logic [15:0] ard_gp_oe;
  logic [15:0] ard_gp_o;

  logic [15:0] pmod_gp_oe;
  logic [15:0] pmod_gp_o;

  // R-Pi header GPIO
  assign rph_g4  = rp_gp_oe[0]  ? rp_gp_o[0]  : 1'bZ;
  assign rph_g5  = rp_gp_oe[1]  ? rp_gp_o[1]  : 1'bZ;
  assign rph_g6  = rp_gp_oe[2]  ? rp_gp_o[2]  : 1'bZ;
  assign rph_g12 = rp_gp_oe[3]  ? rp_gp_o[3]  : 1'bZ;
  assign rph_g13 = rp_gp_oe[4]  ? rp_gp_o[4]  : 1'bZ;
  assign rph_g22 = rp_gp_oe[5]  ? rp_gp_o[5]  : 1'bZ;
  assign rph_g23 = rp_gp_oe[6]  ? rp_gp_o[6]  : 1'bZ;
  assign rph_g24 = rp_gp_oe[7]  ? rp_gp_o[7]  : 1'bZ;
  assign rph_g25 = rp_gp_oe[8]  ? rp_gp_o[8]  : 1'bZ;
  assign rph_g26 = rp_gp_oe[9]  ? rp_gp_o[9]  : 1'bZ;
  assign rph_g27 = rp_gp_oe[10] ? rp_gp_o[10] : 1'bZ;

  // Arduino Shield GPIO
  assign ah_tmpio0 = ard_gp_oe[0] ? ard_gp_o[0] : 1'bZ;
  assign ah_tmpio1 = ard_gp_oe[1] ? ard_gp_o[1] : 1'bZ;
  assign ah_tmpio2 = ard_gp_oe[2] ? ard_gp_o[2] : 1'bZ;
  assign ah_tmpio3 = ard_gp_oe[3] ? ard_gp_o[3] : 1'bZ;
  assign ah_tmpio4 = ard_gp_oe[4] ? ard_gp_o[4] : 1'bZ;
  assign ah_tmpio5 = ard_gp_oe[5] ? ard_gp_o[5] : 1'bZ;
  assign ah_tmpio6 = ard_gp_oe[6] ? ard_gp_o[6] : 1'bZ;
  assign ah_tmpio7 = ard_gp_oe[7] ? ard_gp_o[7] : 1'bZ;
  assign ah_tmpio8 = ard_gp_oe[8] ? ard_gp_o[8] : 1'bZ;
  assign ah_tmpio9 = ard_gp_oe[9] ? ard_gp_o[9] : 1'bZ;

  // PMOD GPIO
  assign pmod0[0] = pmod_gp_oe[0]  ? pmod_gp_o[0]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[1]  ? pmod_gp_o[1]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[2]  ? pmod_gp_o[2]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[3]  ? pmod_gp_o[3]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[4]  ? pmod_gp_o[4]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[5]  ? pmod_gp_o[5]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[6]  ? pmod_gp_o[6]  : 1'bZ;
  assign pmod0[0] = pmod_gp_oe[7]  ? pmod_gp_o[7]  : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[8]  ? pmod_gp_o[8]  : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[9]  ? pmod_gp_o[9]  : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[10] ? pmod_gp_o[10] : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[11] ? pmod_gp_o[11] : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[12] ? pmod_gp_o[12] : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[13] ? pmod_gp_o[13] : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[14] ? pmod_gp_o[14] : 1'bZ;
  assign pmod1[0] = pmod_gp_oe[15] ? pmod_gp_o[15] : 1'bZ;

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

  logic rgbled_dout;

  sonata_system #(
    .GpiWidth       ( 14             ),
    .GpoWidth       ( 23             ),
    .PwmWidth       (  1             ),
    .CheriErrWidth  (  9             ),
    .SRAMInitFile   ( SRAMInitFile   ),
    .SysClkFreq     ( SysClkFreq     ),
    .HRClkFreq      ( HRClkFreq      ),
    .EnableHyperram ( EnableHyperram )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_sys),
    .rst_sys_ni     (rst_sys_n),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    // Hyperram clocks
    .clk_hr_i       (clk_hr),
    .clk_hr90p_i    (clk_hr90p),
    .clk_hr3x_i     (clk_hr3x),

    // GPIO
    .gp_i           ({
                      mb9, // mikroBUS Click interrupt
                      user_sw_n, // user switches
                      nav_sw_n // joystick
                    }),
    .gp_o           ({
                      mb0, // mikroBUS Click reset
                      mb1, // mikroBUS Click chip select
                      ah_tmpio10, // Arduino shield chip select
                      rph_g18, rph_g17, rph_g16_ce2, // R-Pi SPI1 chip select
                      rph_g8_ce0, rph_g7_ce1, // R-Pi SPI0 chip select
                      ethmac_rst, ethmac_cs, // Ethernet
                      appspi_cs, // Flash
                      usrLed, // User LEDs (8 bits)
                      lcd_backlight, lcd_dc, lcd_rst, lcd_cs // LCD screen
                    }),

    // R-Pi Header GPIO
    .rp_gp_i        ({
                      rph_27,
                      rph_26,
                      rph_25,
                      rph_24,
                      rph_23,
                      rph_22,
                      rph_13,
                      rph_12,
                      rph_6,
                      rph_5,
                      rph_4
                    }),
    .rp_gp_o        ({rp_gp_oe, rp_gp_o}),

    // Arduino Shield GPIO
    .ard_gp_i       ({
                      ah_tmpio9,
                      ah_tmpio8,
                      ah_tmpio7,
                      ah_tmpio6,
                      ah_tmpio5,
                      ah_tmpio4,
                      ah_tmpio3,
                      ah_tmpio2,
                      ah_tmpio1,
                      ah_tmpio0
                    }),
    .ard_gp_o       ({ard_gp_oe, ard_gp_o}),

    // PMOD GPIO
    .pmod_gp_i      ({pmod1, pmod0}),
    .pmod_gp_o       ({pmod_gp_oe, pmod_gp_o}),

    // Arduino Shield Analog(ue)
    .ard_an_di_i    (ard_an_di),
    .ard_an_p_i     (ard_an_p),
    .ard_an_n_i     (ard_an_n),

    // UART 0
    .uart0_rx_i     (ser0_rx),
    .uart0_tx_o     (ser0_tx),

    // UART 1
    .uart1_rx_i     (ser1_rx),
    .uart1_tx_o     (ser1_tx),

    // UART 2
    .uart2_rx_i     (rph_rxd0),
    .uart2_tx_o     (rph_txd0),

    // UART 3
    .uart3_rx_i     (mb8),
    .uart3_tx_o     (mb7),

    // UART 4
    .uart4_rx_i     (rs232_rx),
    .uart4_tx_o     (rs232_tx),

    // PWM
    .pwm_o({mb10}),

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
    .spi_ard_rx_i   (ah_tmpio12), // CIPO
    .spi_ard_tx_o   (ah_tmpio11), // COPI
    .spi_ard_sck_o  (ah_tmpio13), // SCLK

    // SPI on mikroBUS Click
    .spi_mkr_rx_i   (mb3), // CIPO
    .spi_mkr_tx_o   (mb4), // COPI
    .spi_mkr_sck_o  (mb2), // SCLK

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
    .td_o,

    .rgbled_dout_o(rgbled_dout),

    .hyperram_dq,
    .hyperram_rwds,
    .hyperram_ckp,
    .hyperram_ckn,
    .hyperram_nrst,
    .hyperram_cs
  );

  assign rgbled0 = ~rgbled_dout;

  // Tie flash wp_n and hold_n to 1 as they're active low and we don't need either signal
  assign appspi_d2 = 1'b1;
  assign appspi_d3 = 1'b1;

  assign led_cheri = cheri_en;
  assign led_legacy = ~cheri_en;
  assign led_halted = 1'b0;

  // Produce 50 MHz system clock from 25 MHz Sonata board clock.
  clkgen_sonata #(
    .SysClkFreq(SysClkFreq),
    .HRClkFreq (HRClkFreq)
  ) u_clkgen(
    .IO_CLK    (mainClk),
    .IO_CLK_BUF(main_clk_buf),
    .clk_sys,
    .clk_usb,
    .clk_hr,
    .clk_hr90p,
    .clk_hr3x,
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
endmodule
