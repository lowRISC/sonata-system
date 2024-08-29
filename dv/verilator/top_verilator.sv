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
  localparam bit EnableCHERI         = 1'b1;
  // Number of CHERI error LEDs.
  localparam int unsigned CheriErrWidth = 9;
  // The symbolic file descriptors are presently unknown to Verilator
  // (described in IEEE 1800-2012).
  localparam int unsigned STDERR = 32'h8000_0002;

  logic unused_uart[3];

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

  // Input clocks and data from the I2C buses.
  wire scl0_in;
  wire scl1_in;
  wire sda0_in;
  wire sda1_in;

  wire unused_ = uart_aux_tx;

  // Simplified clocking scheme for simulations.
  wire clk_usb   = clk_i;
  wire rst_usb_n = rst_ni;

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
  wire cheri_en;
  initial begin
    if (cheri_en) $display("Running with CHERI enabled");
    else $display("Running in legacy software mode");
  end
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

  logic spi_rx[SPI_NUM];
  logic spi_tx[SPI_NUM];
  logic spi_sck[SPI_NUM];

  assign spi_rx[0] = appspi_d1;
  assign appspi_d0 = spi_tx[0];
  assign lcd_copi = spi_tx[1];
  assign appspi_clk = spi_sck[0];
  assign lcd_clk = spi_sck[1];

  // Instantiating the Sonata System.
  // TODO instantiate this with only two UARTs and no SPI when bus is
  // parameterized.
  sonata_system #(
    .GpiWidth        ( 14              ),
    .GpoWidth        ( 23              ),
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
    .clk_hr_i   (1'b0),
    .clk_hr90p_i(1'b0),
    .clk_hr3x_i (1'b0),

    .gp_i         (0),
    .gp_o         ({
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
    .pwm_o        ( ),
    .gp_headers_i ('{default: '0}),
    .gp_headers_o ( ),

    // Arduino Shield Analog(ue)
    .ard_an_di_i    (0),
    .ard_an_p_i     (0),
    .ard_an_n_i     (0),

    // UARTs
    .uart_rx_i     ('{uart_sys_rx, uart_aux_rx, 0, 0, 0}),
    .uart_tx_o     ('{uart_sys_tx, uart_aux_tx, unused_uart[0], unused_uart[1], unused_uart[2]}),

    // SPI hosts
    .spi_rx_i (spi_rx),
    .spi_tx_o (spi_tx),
    .spi_sck_o(spi_sck),
    .spi_eth_irq_ni(1'b1),

    .cheri_en_i (EnableCHERI),
    // CHERI output
    .cheri_err_o    (cheri_err),
    .cheri_en_o     (cheri_en),

    // I2C buses
    .i2c_scl_i     ('{scl0_in, scl1_in}),
    .i2c_scl_o     ('{scl0_o,  scl1_o}),
    .i2c_scl_en_o  ('{scl0_oe, scl1_oe}),
    .i2c_sda_i     ('{sda0_in, sda1_in}),
    .i2c_sda_o     ('{sda0_o,  sda1_o}),
    .i2c_sda_en_o  ('{sda0_oe, sda1_oe}),

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

    .tl_pinmux_o (),
    .tl_pinmux_i (tlul_pkg::TL_D2H_DEFAULT)
  );

  // I2C 0 DPI
  i2cdpi #(
    .ID   ("i2c0")
  ) u_i2c0dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl0_out),
    .sda_i    (sda0_out),
    .scl_o    (scl0_in),
    .sda_o    (sda0_in),
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
    .scl_o    (scl1_in),
    .sda_o    (sda1_in),
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
endmodule
