// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module clkgen_sonata  #(
  // System Clock Frequency is parameterised, allowing it to be adjusted.
  parameter int unsigned SysClkFreq = 50_000_000,
  parameter int unsigned HRClkFreq  = 100_000_000,

  // Frequency of IO_CLK input on the FPGA board.
  parameter int unsigned IOClkFreq = 25_000_000
) (
    // Board clock signal
    input IO_CLK,
    output IO_CLK_BUF,

    // System clock
    output clk_sys,

    // USBDEV clock
    output clk_usb,

    output clk_hr,
    output clk_hr90p,
    output clk_hr3x,

    // Indication that the PLL has stabilized and locked
    output locked
);
  // Required frequency of clk_usb output
  // - usbdev employs x4 oversampling for a 12Mbps Full Speed connection
  localparam int USBClkFreq = 48_000_000;

  logic io_clk_buf;
  logic clk_sys_buf;
  logic clk_sys_unbuf;
  logic clk_fb_buf;
  logic clk_fb_unbuf;
  logic clk_usb_buf;
  logic clk_usb_unbuf;
  logic clk_hr_buf;
  logic clk_hr_unbuf;
  logic clk_hr90p_buf;
  logic clk_hr90p_unbuf;
  logic clk_hr3x_buf;
  logic clk_hr3x_unbuf;

  // Input buffer
  IBUF io_clk_ibuf(
    .I (IO_CLK),
    .O (io_clk_buf)
  );

  PLLE2_ADV #(
    .BANDWIDTH            ("OPTIMIZED"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT        (48),
    .CLKFBOUT_PHASE       (0.000),
    // clk_sys output
    .CLKOUT0_DIVIDE       ((48 * IOClkFreq) / SysClkFreq),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    // clk_usb output
    .CLKOUT1_DIVIDE       ((48 * IOClkFreq) / USBClkFreq),
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),

    .CLKOUT2_DIVIDE       ((48 * IOClkFreq) / HRClkFreq),
    .CLKOUT2_PHASE        (0.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),

    .CLKOUT3_DIVIDE       ((48 * IOClkFreq) / HRClkFreq),
    .CLKOUT3_PHASE        (90.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),

    .CLKOUT4_DIVIDE       ((48 * IOClkFreq) / (HRClkFreq * 3)),
    .CLKOUT4_PHASE        (0.000),
    .CLKOUT4_DUTY_CYCLE   (0.500),

    .CLKIN1_PERIOD        (40.000)
  ) pll (
    .CLKFBOUT            (clk_fb_unbuf),
    .CLKOUT0             (clk_sys_unbuf),
    .CLKOUT1             (clk_usb_unbuf),
    .CLKOUT2             (clk_hr_unbuf),
    .CLKOUT3             (clk_hr90p_unbuf),
    .CLKOUT4             (clk_hr3x_unbuf),
    .CLKOUT5             (),
     // Input clock control
    .CLKFBIN             (clk_fb_buf),
    .CLKIN1              (io_clk_buf),
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
    .DRDY                (),
    .DWE                 (1'b0),
    // Other control and status signals
    .LOCKED              (locked),
    .PWRDWN              (1'b0),
    // Do not reset PLL on external reset, otherwise ILA disconnects at a reset
    .RST                 (1'b0));

  // Output buffering
  BUFG clk_fb_bufg (
    .I (clk_fb_unbuf),
    .O (clk_fb_buf)
  );

  BUFG clk_sys_bufg (
    .I (clk_sys_unbuf),
    .O (clk_sys_buf)
  );

  BUFG clk_usb_bufg (
    .I (clk_usb_unbuf),
    .O (clk_usb_buf)
  );

  BUFG clk_hr_bufg (
    .I (clk_hr_unbuf),
    .O (clk_hr_buf)
  );

  BUFG clk_hr90p_bufg (
    .I (clk_hr90p_unbuf),
    .O (clk_hr90p_buf)
  );

  BUFG clk_hr3x_bufg (
    .I (clk_hr3x_unbuf),
    .O (clk_hr3x_buf)
  );

  // Clock outputs
  assign IO_CLK_BUF = io_clk_buf;
  assign clk_sys    = clk_sys_buf;
  assign clk_usb    = clk_usb_buf;
  assign clk_hr     = clk_hr_buf;
  assign clk_hr90p  = clk_hr90p_buf;
  assign clk_hr3x   = clk_hr3x_buf;

endmodule
