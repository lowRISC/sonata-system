// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level that connects the system to the virtual devices.
module top_verilator (input logic clk_i, rst_ni);
  import sonata_pkg::*;

  parameter bit DisableHyperram = 1'b0;

  // System clock frequency.
  localparam int unsigned SysClkFreq = 30_000_000;
  // HyperRAM clock frequency.
  localparam int unsigned HRClkFreq  = 100_000_000;
  localparam int unsigned BaudRate   = 921_600;
  // Number of CHERI error LEDs.
  localparam int unsigned CheriErrWidth = 9;
  // The symbolic file descriptors are presently unknown to Verilator
  // (described in IEEE 1800-2012).
  localparam int unsigned STDERR = 32'h8000_0002;

  logic uart_sys_rx, uart_sys_tx;
  logic uart_aux_rx, uart_aux_tx;
  assign uart_aux_rx = 1'b1;

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;

  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Output clocks and data to the I2C buses.
  wire scl0_out = scl0_oe ? scl0_o : 1'b1;
  wire scl1_out = scl1_oe ? scl1_o : 1'b1;
  wire sda0_out = sda0_oe ? sda0_o : 1'b1;
  wire sda1_out = sda1_oe ? sda1_o : 1'b1;

  // Clocks and data from the I2C DPI models.
  wire scl0_dpi;
  wire scl1_dpi;
  wire sda0_dpi;
  wire sda1_dpi;

  // Input clocks and data from the I2C buses.
  wire scl0_in = scl0_out & scl0_dpi;
  wire scl1_in = scl1_out & scl1_dpi;
  wire sda0_in = sda0_out & sda0_dpi;
  wire sda1_in = sda1_out & sda1_dpi;

  wire unused_ = uart_aux_tx;

  // Simplified clocking scheme for simulations.
  wire clk_usb   = clk_i;
  wire rst_usb_n = rst_ni;
  wire rst_hr_n  = rst_ni;

  // In Verilator simulation where tri-stated drivers, pullups/pulldowns and drive strengths are
  // not available, the USBDPI model is connected directly to the two-state inputs and outputs of
  // USBDEV itself, i.e. these are not actual USB signals but rather two separated unidirectional
  // buses.
  //
  // USB signals into the USB device from the DPI/host model; these model the VBUS/SENSE and data
  // signals from the on-board TUSB1106 USB transceiver.
  wire usb_sense;
  wire usb_dp_p2d; // D+, differential signaling.
  wire usb_dn_p2d; // D-
  wire usb_d_p2d;  // D, simulated output from differential receiver.

  // USB signals into the DPI/host model from the USB device; these model the signals from the
  // Sonata FPGA to the on-board TUSB1106 transceiver, as well as permitting the differential
  // receiver enable/disable to be tested.
  wire usb_dp_d2p; // D+, differential signaling.
  wire usb_dn_d2p; // D-
  wire usb_dp_en_d2p; // D+ driver enable.
  wire usb_dn_en_d2p; // D- driver enable.
  wire usb_rx_enable; // Enable differential receiver.
  wire usb_dp_pullup; // D+ pullup enable.
  wire usb_dn_pullup; // D- pullup enable.

  // SPI flash interface.
  wire appspi_clk;
  wire appspi_d0; // COPI (controller output peripheral input)
  wire appspi_d1; // CIPO (controller input peripheral output)
  wire appspi_d2; // WP_N (write protect negated)
  wire appspi_d3; // HOLD_N or RESET_N
  wire appspi_cs; // Chip select negated

  // Tie flash wp_n and hold_n to 1 as they're active low and we don't need either signal
  assign appspi_d2 = 1'b1;
  assign appspi_d3 = 1'b1;

  // LCD interface.
  wire lcd_rst;
  wire lcd_dc;
  // SPI interface to LCD.
  wire lcd_copi;
  wire lcd_clk;
  wire lcd_cs;
  // LCD backlight on/off.
  wire lcd_backlight;

  // mikroBUS Click.
  wire mb0, mb1;
  // Arduino
  wire ah_tmpio10;
  // RPi header.
  wire rph_g18, rph_g17, rph_g16_ce2, rph_g8_ce0, rph_g7_ce1;
  // Ethernet
  wire ethmac_rst, ethmac_cs;
  // User LEDs.
  wire [7:0] usrLed;
  // None of these signals is used presently.
  wire unused_io_ = ^{mb0, mb1, ah_tmpio10, rph_g18, rph_g17,
                      rph_g16_ce2, rph_g8_ce0, rph_g7_ce1, ethmac_rst, ethmac_cs,
                      usrLed};

  // Reporting of CHERI enable/disable and any exceptions that occur.
  wire  [CheriErrWidth-1:0] cheri_err;
  logic [CheriErrWidth-1:0] cheri_errored;
  logic cheri_en;

  initial begin : cheri_en_set_and_report
    if ($test$plusargs("disable_cheri")) begin
      cheri_en = 1'b0;
      $display("Running in legacy software mode");
    end else begin
      cheri_en = 1'b1;
      $display("Running with CHERI enabled");
    end
  end : cheri_en_set_and_report

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cheri_errored <= '0;
    else if (|(cheri_err & ~cheri_errored)) begin : cheri_err_reporting
      // Report the first occurrence of each exception by name.
      for (int unsigned e = 0; e < CheriErrWidth; e++) begin
        if (cheri_err[e] & !cheri_errored[e]) begin
          string name;
          case (e)
            0: name = "Bounds";
            1: name = "Tag";
            2: name = "Seal";
            3: name = "Permit Execute";
            4: name = "Permit Load";
            5: name = "Permit Store";
            6: name = "Permit Store Cap";
            7: name = "Permit Store Local Cap";
            8: name = "Permit Acc Sys Regs";
            default: name = "Unknown";
          endcase
          // Ensure that the output is visible promptly.
          $fdisplay(STDERR, "*** CHERI '%s' violation occurred *** at time %t", name, $time);
          $fflush(STDERR);
          // Remember that this error occurred; each error signal will be asserted many times
          // because they are intended to drive LEDs on the FPGA board and are thus modulated.
          cheri_errored <= cheri_errored | cheri_err;
        end
      end
    end
  end

  logic [SPI_CS_NUM-1:0] spi_cs[SPI_NUM];

  sonata_in_pins_t in_from_pins;
  sonata_out_pins_t out_to_pins;
  sonata_inout_pins_t inout_from_pins, inout_to_pins, inout_to_pins_en;

  assign uart_sys_tx = out_to_pins[OUT_PIN_SER0_TX];
  assign uart_aux_tx = out_to_pins[OUT_PIN_SER1_TX];
  assign appspi_d0   = out_to_pins[OUT_PIN_APPSPI_D0];
  assign lcd_copi    = out_to_pins[OUT_PIN_LCD_COPI];
  assign appspi_clk  = out_to_pins[OUT_PIN_APPSPI_CLK];
  assign lcd_clk     = out_to_pins[OUT_PIN_LCD_CLK];

  assign {scl0_o,  scl0_oe} = {inout_to_pins[INOUT_PIN_SCL0], inout_to_pins_en[INOUT_PIN_SCL0]};
  assign {scl1_o,  scl1_oe} = {inout_to_pins[INOUT_PIN_SCL1], inout_to_pins_en[INOUT_PIN_SCL1]};
  assign {sda0_o,  sda0_oe} = {inout_to_pins[INOUT_PIN_SDA0], inout_to_pins_en[INOUT_PIN_SDA0]};
  assign {sda1_o,  sda1_oe} = {inout_to_pins[INOUT_PIN_SDA1], inout_to_pins_en[INOUT_PIN_SDA1]};

  assign in_from_pins[IN_PIN_APPSPI_D1] = appspi_d1;
  assign in_from_pins[IN_PIN_SER0_RX]   = uart_sys_rx;
  assign in_from_pins[IN_PIN_SER1_RX]   = uart_aux_rx;

  assign inout_from_pins[INOUT_PIN_SCL0] = scl0_in;
  assign inout_from_pins[INOUT_PIN_SDA0] = sda0_in;
  assign inout_from_pins[INOUT_PIN_SCL1] = scl1_in;
  assign inout_from_pins[INOUT_PIN_SDA1] = sda1_in;

  // SPI CS outputs from GPIO pins; these are scheduled to be dropped but they are still required
  // by the `gp_o` port presently.
  logic gp_lcd_cs, gp_appspi_cs, gp_ethmac_cs;
  logic gp_rph_g8_ce0, gp_rph_g7_ce1;
  logic gp_rph_g18, gp_rph_g17, gp_rph_g16_ce2;
  logic gp_ah_tmpio10, gp_mb1;

  // CS outputs to SPI peripherals from controllers.
  assign appspi_cs   = spi_cs[0][0];
  assign lcd_cs      = spi_cs[1][0];
  assign ethmac_cs   = spi_cs[2][0];
  assign rph_g8_ce0  = spi_cs[3][0];
  assign rph_g7_ce1  = spi_cs[3][1];
  assign ah_tmpio10  = spi_cs[3][2];
  assign rph_g18     = spi_cs[4][0];
  assign rph_g17     = spi_cs[4][1];
  assign rph_g16_ce2 = spi_cs[4][2];
  assign mb1         = spi_cs[4][3];

  wire unused_gp_spi_cs_ = ^{gp_lcd_cs, gp_appspi_cs, gp_ethmac_cs,
                             gp_rph_g8_ce0, gp_rph_g7_ce1, gp_rph_g18, gp_rph_g17, gp_rph_g16_ce2,
                             gp_ah_tmpio10, gp_mb1};

  logic [8:0] unused_gp_o;

  // Instantiating the Sonata System.
  // TODO instantiate this with only two UARTs and no SPI when bus is
  // parameterized.
  sonata_system #(
    .PwmWidth        (  1              ),
    .CheriErrWidth   ( CheriErrWidth   ),
    .SysClkFreq      ( SysClkFreq      ),
    .HRClkFreq       ( HRClkFreq       ),
    .DisableHyperram ( DisableHyperram )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_i),
    .rst_sys_ni     (rst_ni),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    // SRAM model used for hyperram so no hyperram clock is provided
    .clk_hr_i       (1'b0),
    .clk_hr90p_i    (1'b0),
    .clk_hr3x_i     (1'b0),
    .rst_hr_ni      (rst_hr_n),

    .gp_i           (0),
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

    .gp_o_en      ( ),
    .pwm_o        ( ),

    // Arduino Shield Analog(ue)
    .ard_an_di_i    (0),
    .ard_an_p_i     (0),
    .ard_an_n_i     (0),

    // SPI hosts
    .spi_cs_o       (spi_cs),
    .spi_eth_irq_ni(1'b1),

    // CHERI signals
    .cheri_en_i     (cheri_en ),
    .cheri_err_o    (cheri_err),
    .cheri_en_o     (         ),

    // Reception from USB host via transceiver
    .usb_dp_i         (usb_dp_p2d),
    .usb_dn_i         (usb_dn_p2d),
    .usb_rx_d_i       (usb_d_p2d),

    // Transmission to USB host via transceiver
    .usb_dp_o         (usb_dp_d2p),
    .usb_dp_en_o      (usb_dp_en_d2p),
    .usb_dn_o         (usb_dn_d2p),
    .usb_dn_en_o      (usb_dn_en_d2p),

    // Configuration and control of USB transceiver
    .usb_sense_i      (usb_sense),
    .usb_dp_pullup_o  (usb_dp_pullup),
    .usb_dn_pullup_o  (usb_dn_pullup),
    .usb_rx_enable_o  (usb_rx_enable),

    // User JTAG
    .tck_i  ('0),
    .tms_i  ('0),
    .trst_ni(rst_ni),
    .td_i   ('0),
    .td_o   (),

    .rgbled_dout_o (),

    // SRAM model used for hyperram so don't connect hyperram IO
    .hyperram_dq  (),
    .hyperram_rwds(),
    .hyperram_ckp (),
    .hyperram_ckn (),
    .hyperram_nrst(),
    .hyperram_cs  (),

    .in_from_pins_i     (in_from_pins    ),
    .out_to_pins_o      (out_to_pins     ),
    .out_to_pins_en_o   (                ),
    .inout_from_pins_i  (inout_from_pins ),
    .inout_to_pins_o    (inout_to_pins   ),
    .inout_to_pins_en_o (inout_to_pins_en)
  );

  // I2C 0 DPI
  i2cdpi #(
    .ID   ("i2c0")
  ) u_i2c0dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl0_out),
    .sda_i    (sda0_out),
    .scl_o    (scl0_dpi),
    .sda_o    (sda0_dpi),
    // Out-Of-Band data.
    .oob_in   (1'b0),
    .oob_out  ()  // not used
  );

  // I2C 1 DPI
  i2cdpi #(
    .ID   ("i2c1")
  ) u_i2c1dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl1_out),
    .sda_i    (sda1_out),
    .scl_o    (scl1_dpi),
    .sda_o    (sda1_dpi),
    // Out-Of-Band data.
    .oob_in   (1'b0),
    .oob_out  ()  // not used
  );

  // Virtual UART
  uartdpi #(
    .BAUD ( BaudRate    ),
    .FREQ ( SysClkFreq  )
  ) u_uartdpi (
    .clk_i,
    .rst_ni,
    .active(1'b1       ),
    .tx_o  (uart_sys_rx),
    .rx_i  (uart_sys_tx)
  );

  // USB DPI; simulated USB host.
  usbdpi u_usbdpi (
    .clk_i           (clk_usb),
    .rst_ni          (rst_usb_n),
    .clk_48MHz_i     (clk_usb),
    .enable          (1'b1),
    // D+ drivers and their enables.
    .dp_en_p2d       (),
    .dp_p2d          (usb_dp_p2d),
    .dp_d2p          (usb_dp_d2p),
    .dp_en_d2p       (usb_dp_en_d2p),
    // D- drivers and their enables.
    .dn_en_p2d       (),
    .dn_p2d          (usb_dn_p2d),
    .dn_d2p          (usb_dn_d2p),
    .dn_en_d2p       (usb_dn_en_d2p),
    // D drivers (used when external differential receiver is enabled).
    .d_p2d           (usb_d_p2d),
    .d_d2p           (1'b0),
    .d_en_d2p        (1'b0),
    .se0_d2p         (1'b0),
    // Enable signal for external differential receiver.
    .rx_enable_d2p   (usb_rx_enable),
    // Sonata FPGA does not employ D/SE0 signaling in place of D+/D-.
    .tx_use_d_se0_d2p(1'b0),

    // VBUS/SENSE signal indicating the presence of the USB host.
    .sense_p2d       (usb_sense),
    // Pullup enables from the USB device.
    .pullupdp_d2p    (usb_dp_pullup),
    .pullupdn_d2p    (usb_dn_pullup)
  );

  // SPI connection to flash.
  spidpi #(
    .ID       ("flash"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (2),
    .OOB_OutW (1)
  ) u_spidpi_flash (
    .rst_ni   (rst_ni),

    .sck      (appspi_clk),
    .cs       (appspi_cs),
    .copi     (appspi_d0),
    .cipo     (appspi_d1),

    .oob_in   ({appspi_d3, appspi_d2}),
    .oob_out  ( )
  );

  // SPI connection to LCD.
  spidpi #(
    .ID       ("lcd"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (3),
    .OOB_OutW (1)
  ) u_spidpi_lcd (
    .rst_ni   (rst_ni),

    .sck      (lcd_clk),
    .cs       (lcd_cs),
    .copi     (lcd_copi),
    .cipo     ( ),  // not used.

    .oob_in   ({lcd_dc, lcd_rst, lcd_backlight}),
    .oob_out  ( )  // not used.
  );


  export "DPI-C" function mhpmcounter_get;

  function automatic longint unsigned mhpmcounter_get(int index);
    return u_sonata_system.u_top_tracing.u_ibex_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
  endfunction
endmodule
