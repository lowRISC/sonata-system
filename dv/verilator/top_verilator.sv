// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level that connects the system to the virtual devices.
module top_verilator (input logic clk_i, rst_ni);

  localparam ClockFrequency = 30_000_000;
  localparam BaudRate       = 115_200;
  localparam EnableCHERI    = 1'b1;

  logic uart_sys_rx, uart_sys_tx;

  logic uart_aux_rx, uart_aux_tx;
  assign uart_aux_rx = 1'b1;

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;

  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Nothing else driving the buses at present.
  wire scl0 = scl0_oe ? scl0_o : 1'b1;
  wire scl1 = scl1_oe ? scl0_o : 1'b1;
  wire sda0 = sda0_oe ? sda0_o : 1'b1;
  wire sda1 = sda1_oe ? sda1_o : 1'b1;

  wire unused_ = ^{scl0_o, scl0_oe, sda0_o, sda0_oe,
                   scl1_o, scl1_oe, sda1_o, sda1_oe,
                   uart_aux_tx};

  // Simplified clocking scheme for simulations.
  wire clk_usb   = clk_i;
  wire rst_usb_n = rst_ni;

  // Instantiating the Sonata System.
  sonata_system u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_i),
    .rst_sys_ni     (rst_ni),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    .gp_i     (0),
    .gp_o     ( ),
    .pwm_o    ( ),
    .rp_gp_i  (0),
    .rp_gp_o  ( ),
    .ard_gp_i (0),
    .ard_gp_o ( ),
    .pmod_gp_i(0),
    .pmod_gp_o( ),

    // UART 0 TX and RX
    .uart0_rx_i     (uart_sys_rx),
    .uart0_tx_o     (uart_sys_tx),

    // UART 1 TX and RX
    .uart1_rx_i     (uart_aux_rx),
    .uart1_tx_o     (uart_aux_tx),

    .uart2_rx_i     (),
    .uart2_tx_o     (),

    .uart3_rx_i     (),
    .uart3_tx_o     (),

    .uart4_rx_i     (),
    .uart4_tx_o     (),

    .spi_flash_rx_i (0),
    .spi_flash_tx_o ( ),
    .spi_flash_sck_o( ),

    .spi_lcd_rx_i (0),
    .spi_lcd_tx_o ( ),
    .spi_lcd_sck_o( ),

    .spi_eth_rx_i  (0),
    .spi_eth_tx_o  ( ),
    .spi_eth_sck_o ( ),
    .spi_eth_irq_ni(1'b1),

    .spi_rp0_rx_i (0),
    .spi_rp0_tx_o ( ),
    .spi_rp0_sck_o( ),

    .spi_rp1_rx_i (0),
    .spi_rp1_tx_o ( ),
    .spi_rp1_sck_o( ),

    .spi_ard_rx_i (0),
    .spi_ard_tx_o ( ),
    .spi_ard_sck_o( ),

    .spi_mkr_rx_i (0),
    .spi_mkr_tx_o ( ),
    .spi_mkr_sck_o( ),

    .cheri_en_i (EnableCHERI),
    // CHERI output
    .cheri_err_o(),
    .cheri_en_o (),

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
    .usb_dp_i         (1'b0),
    .usb_dn_i         (1'b0),
    .usb_rx_d_i       (1'b0),

    // Transmission to USB host via transceiver
    .usb_dp_o         (),
    .usb_dp_en_o      (),
    .usb_dn_o         (),
    .usb_dn_en_o      (),

    // Configuration and control of USB transceiver
    .usb_sense_i      (1'b0),
    .usb_dp_pullup_o  (),
    .usb_dn_pullup_o  (),
    .usb_rx_enable_o  (),

    // User JTAG
    .tck_i  ('0),
    .tms_i  ('0),
    .trst_ni(rst_ni),
    .td_i   ('0),
    .td_o   (),

    .rgbled_dout_o ()
  );

  // Virtual UART
  uartdpi #(
    .BAUD ( BaudRate       ),
    .FREQ ( ClockFrequency )
  ) u_uartdpi (
    .clk_i,
    .rst_ni,
    .active(1'b1       ),
    .tx_o  (uart_sys_rx),
    .rx_i  (uart_sys_tx)
  );
endmodule
