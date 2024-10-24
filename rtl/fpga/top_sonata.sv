// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Sonata system top level for the Sonata PCB
module top_sonata
  import sonata_pkg::*;
(
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
  input  logic [2:0] selSw,

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
  inout  logic       rph_g11_sclk, // SPI0
  inout  logic       rph_g10_copi, // SPI0
  inout  logic       rph_g9_cipo,  // SPI0
  output logic       rph_g8_ce0,   // SPI0
  output logic       rph_g7_ce1,   // SPI0

  inout  logic       rph_g21_sclk, // SPI1
  inout  logic       rph_g20_copi, // SPI1
  inout  logic       rph_g19_cipo, // SPI1
  output logic       rph_g18,      // SPI1 CE0
  output logic       rph_g17,      // SPI1 CE1
  output logic       rph_g16_ce2,  // SPI1

  // R-Pi header UART
  inout  logic       rph_txd0,
  inout  logic       rph_rxd0,

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
  inout  logic       ah_tmpio11, // COPI
  inout  logic       ah_tmpio12, // CIPO or GP
  inout  logic       ah_tmpio13, // SCLK

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

  // MicroSD card slot
  output logic       microsd_clk,  // SPI mode: SCLK
  input  logic       microsd_dat0, // SPI mode: CIPO
//input  logic       microsd_dat1, // SPI mode: NC
//input  logic       microsd_dat2, // SPI mode: NC
  output logic       microsd_dat3, // SPI mode: CS_N
  output logic       microsd_cmd,  // SPI mode: COPI
  input  logic       microsd_det,  // Card insertion detection

  inout  wire [7:0]  hyperram_dq,
  inout  wire        hyperram_rwds,
  output wire        hyperram_ckp,
  output wire        hyperram_ckn,
  output wire        hyperram_nrst,
  output wire        hyperram_cs
);
  import sonata_pkg::*;

  // System clock frequency.
  parameter int unsigned SysClkFreq = 30_000_000;
  parameter int unsigned HRClkFreq  = 100_000_000;

  parameter SRAMInitFile    = "";
  parameter DisableHyperram = 1'b0;

  // Main/board clock and reset
  logic main_clk_buf;
  logic rst_n;

  // System clock and reset
  logic clk_sys;
  logic rst_sys_n;

  // USB device clock and reset
  logic clk_usb;
  logic rst_usb_n;

  // HyperRAM clocks and reset
  logic clk_hr, clk_hr90p, clk_hr3x;
  logic rst_hr_n;

  logic [7:0] reset_counter;
  logic pll_locked;
  logic rst_btn;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;
  logic [2:0] sel_sw_n;

  assign led_bootok = rst_sys_n;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~navSw;
  assign user_sw_n = ~usrSw;
  assign sel_sw_n = ~selSw;

  assign usrusb_spd = 1'b1;  // Full Speed operation.

  logic dp_en_d2p;
  logic rx_enable_d2p;
  assign usrusb_oe  = !dp_en_d2p;  // Active low Output Enable.
  assign usrusb_sus = !rx_enable_d2p;

  sonata_in_pins_t in_from_pins;
  sonata_out_pins_t out_to_pins, out_to_pins_en;
  sonata_inout_pins_t inout_from_pins, inout_to_pins, inout_to_pins_en;

  wire sonata_out_pins_t   output_pins;

  logic cheri_en;

  logic [SPI_CS_NUM-1:0] spi_cs[SPI_NUM];

  // SPI CS outputs from GPIO pins; these are scheduled to be dropped but they are still required
  // by the `gp_o` port presently.
  logic gp_lcd_cs, gp_appspi_cs, gp_ethmac_cs;
  logic gp_rph_g8_ce0, gp_rph_g7_ce1;
  logic gp_rph_g18, gp_rph_g17, gp_rph_g16_ce2;
  logic gp_ah_tmpio10, gp_mb1;

  // CS outputs to SPI peripherals from controllers.
  assign appspi_cs    = spi_cs[0][0];
  assign lcd_cs       = spi_cs[1][0];
  assign ethmac_cs    = spi_cs[2][0];
  assign rph_g8_ce0   = spi_cs[3][0];
  assign rph_g7_ce1   = spi_cs[3][1];
  assign ah_tmpio10   = spi_cs[3][2];
  assign microsd_dat3 = spi_cs[3][3];
  assign rph_g18      = spi_cs[4][0];
  assign rph_g17      = spi_cs[4][1];
  assign rph_g16_ce2  = spi_cs[4][2];
  assign mb1          = spi_cs[4][3];

  wire unused_gp_spi_cs_ = ^{gp_lcd_cs, gp_appspi_cs, gp_ethmac_cs,
                             gp_rph_g8_ce0, gp_rph_g7_ce1,
                             gp_rph_g18, gp_rph_g17, gp_rph_g16_ce2,
                             gp_ah_tmpio10, gp_mb1};

  // Collect the unused CS lines.
  wire unused_spi_cs_ = ^{spi_cs[0][SPI_CS_NUM-1:1],
                          spi_cs[1][SPI_CS_NUM-1:1],
                          spi_cs[2][SPI_CS_NUM-1:1]};

  // Enable CHERI by default.
  logic enable_cheri;
  assign enable_cheri = 1'b1;

  logic rgbled_dout;
  logic [8:0] unused_gp_o;

  sonata_system #(
    .PwmWidth        (  1             ),
    .CheriErrWidth   (  9             ),
    .SRAMInitFile    ( SRAMInitFile   ),
    .SysClkFreq      ( SysClkFreq     ),
    .HRClkFreq       ( HRClkFreq      ),
    .DisableHyperram ( DisableHyperram )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_sys),
    .rst_sys_ni     (rst_sys_n),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    // HyperRAM clocks and reset
    .clk_hr_i       (clk_hr),
    .clk_hr90p_i    (clk_hr90p),
    .clk_hr3x_i     (clk_hr3x),
    .rst_hr_ni      (rst_hr_n),

    // GPIO
    .gp_i           ({
                      14'b0,
                      microsd_det, // MicroSD card insertion detection
                      sel_sw_n, // Software selection switches
                      mb9, // mikroBUS Click interrupt
                      user_sw_n, // user switches
                      nav_sw_n // joystick
                    }),
    .gp_o           ({
                      unused_gp_o,
                      mb0, // mikroBUS Click reset
                      gp_mb1, // mikroBUS Click chip select
                      gp_ah_tmpio10, // Arduino shield chip select
                      gp_rph_g16_ce2, gp_rph_g17, gp_rph_g18, // R-Pi SPI1 chip selects [2:0]
                      gp_rph_g7_ce1, gp_rph_g8_ce0, // R-Pi SPI0 chip selects [1:0]
                      ethmac_rst, gp_ethmac_cs, // Ethernet
                      gp_appspi_cs, // Flash
                      usrLed, // User LEDs (8 bits)
                      lcd_backlight, lcd_dc, lcd_rst, gp_lcd_cs // LCD screen
                    }),
    .gp_o_en        (),

    // Arduino Shield Analog(ue)
    .ard_an_di_i    (ard_an_di),
    .ard_an_p_i     (ard_an_p),
    .ard_an_n_i     (ard_an_n),

    // PWM
    .pwm_o({mb10}),

    // SPI Chip Selects
    .spi_cs_o       (spi_cs),

    // Interrupt for Ethernet is out of band
    .spi_eth_irq_ni (ethmac_intr),

    // CHERI signals
    .cheri_en_i     (enable_cheri),
    .cheri_err_o    (cheriErr),
    .cheri_en_o     (cheri_en),

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
    .trst_ni(rst_n),
    .td_i,
    .td_o,

    .rgbled_dout_o(rgbled_dout),

    .hyperram_dq,
    .hyperram_rwds,
    .hyperram_ckp,
    .hyperram_ckn,
    .hyperram_nrst,
    .hyperram_cs,

    .in_from_pins_i     (in_from_pins    ),
    .out_to_pins_o      (out_to_pins     ),
    .out_to_pins_en_o   (out_to_pins_en  ),
    .inout_from_pins_i  (inout_from_pins ),
    .inout_to_pins_o    (inout_to_pins   ),
    .inout_to_pins_en_o (inout_to_pins_en)
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
    .rst_no      (rst_n)
  );

  // Synchronise reset signals for other clock domains
  rst_sync u_rst_sync (
    .clk_sys_i  (clk_sys),
    .clk_usb_i  (clk_usb),
    .clk_hr_i   (clk_hr),
    .rst_ni     (rst_n),
    .rst_sys_no (rst_sys_n),
    .rst_usb_no (rst_usb_n),
    .rst_hr_no  (rst_hr_n)
  );

  // Input Pins
  assign in_from_pins[IN_PIN_MICROSD_DAT0] = microsd_dat0;
  assign in_from_pins[IN_PIN_MB8         ] = mb8;
  assign in_from_pins[IN_PIN_MB3         ] = mb3;
  assign in_from_pins[IN_PIN_ETHMAC_CIPO ] = ethmac_cipo;
  assign in_from_pins[IN_PIN_APPSPI_D1   ] = appspi_d1;
  assign in_from_pins[IN_PIN_RS232_RX    ] = rs232_rx;
  assign in_from_pins[IN_PIN_SER1_RX     ] = ser1_rx;
  assign in_from_pins[IN_PIN_SER0_RX     ] = ser0_rx;

  // Output Pins
  // pull output pins low when their output isn't enabled.
  assign output_pins = out_to_pins_en & out_to_pins;

  assign microsd_cmd = output_pins[OUT_PIN_MICROSD_CMD];
  assign microsd_clk = output_pins[OUT_PIN_MICROSD_CLK];
  assign mb7         = output_pins[OUT_PIN_MB7        ];
  assign mb4         = output_pins[OUT_PIN_MB4        ];
  assign mb2         = output_pins[OUT_PIN_MB2        ];
  assign ethmac_sclk = output_pins[OUT_PIN_ETHMAC_SCLK];
  assign ethmac_copi = output_pins[OUT_PIN_ETHMAC_COPI];
  assign lcd_clk     = output_pins[OUT_PIN_LCD_CLK    ];
  assign lcd_copi    = output_pins[OUT_PIN_LCD_COPI   ];
  assign appspi_clk  = output_pins[OUT_PIN_APPSPI_CLK ];
  assign appspi_d0   = output_pins[OUT_PIN_APPSPI_D0  ];
  assign rs232_tx    = output_pins[OUT_PIN_RS232_TX   ];
  assign ser1_tx     = output_pins[OUT_PIN_SER1_TX    ];
  assign ser0_tx     = output_pins[OUT_PIN_SER0_TX    ];

  // Inout Pins
  localparam int unsigned USED_INOUT_PIN_NUM = INOUT_PIN_NUM-9;
  logic [USED_INOUT_PIN_NUM-1:0] used_inout_to_pins = {
    inout_to_pins[INOUT_PIN_PMOD1_7:INOUT_PIN_MB5],
    inout_to_pins[INOUT_PIN_AH_TMPIO16],
    // TODO connect ah_tmpio{14,15,17} through XDC
    inout_to_pins[INOUT_PIN_AH_TMPIO13:INOUT_PIN_AH_TMPIO11],
    // ah_tmpio10 connected in manual GPIO.
    inout_to_pins[INOUT_PIN_AH_TMPIO9:INOUT_PIN_RPH_G19_CIPO],
    // rph_g16_ce2, rph_g17, rph_g18 connected in manual GPIO.
    inout_to_pins[INOUT_PIN_RPH_RXD0:INOUT_PIN_RPH_G9_CIPO],
    // rph_g7_ce1, rph_g8_ce0 connected in manual GPIO
    inout_to_pins[INOUT_PIN_RPH_G6:INOUT_PIN_SCL0]
  };
  logic [USED_INOUT_PIN_NUM-1:0] used_inout_to_pins_en = {
    inout_to_pins_en[INOUT_PIN_PMOD1_7:INOUT_PIN_MB5],
    inout_to_pins_en[INOUT_PIN_AH_TMPIO16],
    // TODO connect ah_tmpio{14,15,17} through XDC
    inout_to_pins_en[INOUT_PIN_AH_TMPIO13:INOUT_PIN_AH_TMPIO11],
    // ah_tmpio10 connected in manual GPIO.
    inout_to_pins_en[INOUT_PIN_AH_TMPIO9:INOUT_PIN_RPH_G19_CIPO],
    // rph_g16_ce2, rph_g17, rph_g18 connected in manual GPIO.
    inout_to_pins_en[INOUT_PIN_RPH_RXD0:INOUT_PIN_RPH_G9_CIPO],
    // rph_g7_ce1, rph_g8_ce0 connected in manual GPIO
    inout_to_pins_en[INOUT_PIN_RPH_G6:INOUT_PIN_SCL0]
  };
  logic [USED_INOUT_PIN_NUM-1:0] used_inout_from_pins;
  assign {
    inout_from_pins[INOUT_PIN_PMOD1_7:INOUT_PIN_MB5],
    inout_from_pins[INOUT_PIN_AH_TMPIO16],
    // TODO connect ah_tmpio{14,15,17} through XDC
    inout_from_pins[INOUT_PIN_AH_TMPIO13:INOUT_PIN_AH_TMPIO11],
    // ah_tmpio10 connected in manual GPIO.
    inout_from_pins[INOUT_PIN_AH_TMPIO9:INOUT_PIN_RPH_G19_CIPO],
    // rph_g16_ce2, rph_g17, rph_g18 connected in manual GPIO.
    inout_from_pins[INOUT_PIN_RPH_RXD0:INOUT_PIN_RPH_G9_CIPO],
    // rph_g7_ce1, rph_g8_ce0 connected in manual GPIO
    inout_from_pins[INOUT_PIN_RPH_G6:INOUT_PIN_SCL0]
  } = used_inout_from_pins;

  padring #(
    .InoutNumber(USED_INOUT_PIN_NUM)
  ) u_padring (
    .inout_to_pins_i   (used_inout_to_pins   ),
    .inout_to_pins_en_i(used_inout_to_pins_en),
    .inout_from_pins_o (used_inout_from_pins ),
    .inout_pins_io({
      pmod1,
      pmod0,
      mb6,
      mb5,
      ah_tmpio16,
      ah_tmpio13,
      ah_tmpio12,
      ah_tmpio11,
      ah_tmpio9,
      ah_tmpio8,
      ah_tmpio7,
      ah_tmpio6,
      ah_tmpio5,
      ah_tmpio4,
      ah_tmpio3,
      ah_tmpio2,
      ah_tmpio1,
      ah_tmpio0,
      rph_g27,
      rph_g26,
      rph_g25,
      rph_g24,
      rph_g23,
      rph_g22,
      rph_g21_sclk,
      rph_g20_copi,
      rph_g19_cipo,
      rph_rxd0,
      rph_txd0,
      rph_g13,
      rph_g12,
      rph_g11_sclk,
      rph_g10_copi,
      rph_g9_cipo,
      rph_g6,
      rph_g5,
      rph_g4,
      rph_g3_scl,
      rph_g2_sda,
      rph_g1,
      rph_g0,
      sda1,
      scl1,
      sda0,
      scl0
    })
  );
endmodule : top_sonata
