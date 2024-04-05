// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level that connects the system to the virtual devices.
module top_verilator (input logic clk_i, rst_ni);

  localparam ClockFrequency = 50_000_000;
  localparam BaudRate       = 115_200;

  logic uart_sys_rx, uart_sys_tx;

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
