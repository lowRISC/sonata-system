// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level that connects the system to the virtual devices.
module top_verilator (input logic clk_i, rst_ni);

  localparam ClockFrequency = 50_000_000;
  localparam BaudRate       = 115_200;

  logic uart_sys_rx, uart_sys_tx;

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
                   scl1_o, scl1_oe, sda1_o, sda1_oe};

  // Instantiating the Sonata System.
  sonata_system u_sonata_system (
    // Clock and Reset
    .clk_sys_i (clk_i ),
    .rst_sys_ni(rst_ni),

    // UART TX and RX
    .uart_rx_i (uart_sys_rx),
    .uart_tx_o (uart_sys_tx),

    // Remaining IO
    .gp_i     (0),
    .gp_o     ( ),
    .pwm_o    ( ),
    .spi_rx_i (0),
    .spi_tx_o ( ),
    .spi_sck_o( ),

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

    // User JTAG
    .tck_i  (),
    .tms_i  (),
    .trst_ni(rst_ni),
    .td_i   (),
    .td_o   ()
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
