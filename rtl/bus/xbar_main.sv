// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// xbar_main module generated by `tlgen.py` tool
// all reset signals should be generated from one reset signal to not make any deadlock
//
// Interconnect
// ibex_lsu
//   -> s1n_24
//     -> sm1_25
//       -> sram
//     -> hyperram
//     -> rev_tag
//     -> gpio
//     -> pinmux
//     -> system_info
//     -> rgbled_ctrl
//     -> hw_rev
//     -> xadc
//     -> timer
//     -> spi_board
//     -> spi_lcd
//     -> pwm0
//     -> uart0
//     -> uart1
//     -> uart2
//     -> i2c0
//     -> i2c1
//     -> spi0
//     -> spi1
//     -> asf_26
//       -> usbdev
//     -> rv_plic
// dbg_host
//   -> sm1_25
//     -> sram

module xbar_main (
  input clk_sys_i,
  input clk_usb_i,
  input rst_sys_ni,
  input rst_usb_ni,

  // Host interfaces
  input  tlul_pkg::tl_h2d_t tl_ibex_lsu_i,
  output tlul_pkg::tl_d2h_t tl_ibex_lsu_o,
  input  tlul_pkg::tl_h2d_t tl_dbg_host_i,
  output tlul_pkg::tl_d2h_t tl_dbg_host_o,

  // Device interfaces
  output tlul_pkg::tl_h2d_t tl_sram_o,
  input  tlul_pkg::tl_d2h_t tl_sram_i,
  output tlul_pkg::tl_h2d_t tl_hyperram_o,
  input  tlul_pkg::tl_d2h_t tl_hyperram_i,
  output tlul_pkg::tl_h2d_t tl_rev_tag_o,
  input  tlul_pkg::tl_d2h_t tl_rev_tag_i,
  output tlul_pkg::tl_h2d_t tl_gpio_o,
  input  tlul_pkg::tl_d2h_t tl_gpio_i,
  output tlul_pkg::tl_h2d_t tl_pinmux_o,
  input  tlul_pkg::tl_d2h_t tl_pinmux_i,
  output tlul_pkg::tl_h2d_t tl_rgbled_ctrl_o,
  input  tlul_pkg::tl_d2h_t tl_rgbled_ctrl_i,
  output tlul_pkg::tl_h2d_t tl_hw_rev_o,
  input  tlul_pkg::tl_d2h_t tl_hw_rev_i,
  output tlul_pkg::tl_h2d_t tl_xadc_o,
  input  tlul_pkg::tl_d2h_t tl_xadc_i,
  output tlul_pkg::tl_h2d_t tl_system_info_o,
  input  tlul_pkg::tl_d2h_t tl_system_info_i,
  output tlul_pkg::tl_h2d_t tl_timer_o,
  input  tlul_pkg::tl_d2h_t tl_timer_i,
  output tlul_pkg::tl_h2d_t tl_spi_board_o,
  input  tlul_pkg::tl_d2h_t tl_spi_board_i,
  output tlul_pkg::tl_h2d_t tl_spi_lcd_o,
  input  tlul_pkg::tl_d2h_t tl_spi_lcd_i,
  output tlul_pkg::tl_h2d_t tl_pwm0_o,
  input  tlul_pkg::tl_d2h_t tl_pwm0_i,
  output tlul_pkg::tl_h2d_t tl_uart0_o,
  input  tlul_pkg::tl_d2h_t tl_uart0_i,
  output tlul_pkg::tl_h2d_t tl_uart1_o,
  input  tlul_pkg::tl_d2h_t tl_uart1_i,
  output tlul_pkg::tl_h2d_t tl_uart2_o,
  input  tlul_pkg::tl_d2h_t tl_uart2_i,
  output tlul_pkg::tl_h2d_t tl_i2c0_o,
  input  tlul_pkg::tl_d2h_t tl_i2c0_i,
  output tlul_pkg::tl_h2d_t tl_i2c1_o,
  input  tlul_pkg::tl_d2h_t tl_i2c1_i,
  output tlul_pkg::tl_h2d_t tl_spi0_o,
  input  tlul_pkg::tl_d2h_t tl_spi0_i,
  output tlul_pkg::tl_h2d_t tl_spi1_o,
  input  tlul_pkg::tl_d2h_t tl_spi1_i,
  output tlul_pkg::tl_h2d_t tl_usbdev_o,
  input  tlul_pkg::tl_d2h_t tl_usbdev_i,
  output tlul_pkg::tl_h2d_t tl_rv_plic_o,
  input  tlul_pkg::tl_d2h_t tl_rv_plic_i,

  input prim_mubi_pkg::mubi4_t scanmode_i
);

  import tlul_pkg::*;
  import tl_main_pkg::*;

  // scanmode_i is currently not used, but provisioned for future use
  // this assignment prevents lint warnings
  logic unused_scanmode;
  assign unused_scanmode = ^scanmode_i;

  tl_h2d_t tl_s1n_24_us_h2d ;
  tl_d2h_t tl_s1n_24_us_d2h ;


  tl_h2d_t tl_s1n_24_ds_h2d [22];
  tl_d2h_t tl_s1n_24_ds_d2h [22];

  // Create steering signal
  logic [4:0] dev_sel_s1n_24;


  tl_h2d_t tl_sm1_25_us_h2d [2];
  tl_d2h_t tl_sm1_25_us_d2h [2];

  tl_h2d_t tl_sm1_25_ds_h2d ;
  tl_d2h_t tl_sm1_25_ds_d2h ;

  tl_h2d_t tl_asf_26_us_h2d ;
  tl_d2h_t tl_asf_26_us_d2h ;
  tl_h2d_t tl_asf_26_ds_h2d ;
  tl_d2h_t tl_asf_26_ds_d2h ;



  assign tl_sm1_25_us_h2d[0] = tl_s1n_24_ds_h2d[0];
  assign tl_s1n_24_ds_d2h[0] = tl_sm1_25_us_d2h[0];

  assign tl_hyperram_o = tl_s1n_24_ds_h2d[1];
  assign tl_s1n_24_ds_d2h[1] = tl_hyperram_i;

  assign tl_rev_tag_o = tl_s1n_24_ds_h2d[2];
  assign tl_s1n_24_ds_d2h[2] = tl_rev_tag_i;

  assign tl_gpio_o = tl_s1n_24_ds_h2d[3];
  assign tl_s1n_24_ds_d2h[3] = tl_gpio_i;

  assign tl_pinmux_o = tl_s1n_24_ds_h2d[4];
  assign tl_s1n_24_ds_d2h[4] = tl_pinmux_i;

  assign tl_system_info_o = tl_s1n_24_ds_h2d[5];
  assign tl_s1n_24_ds_d2h[5] = tl_system_info_i;

  assign tl_rgbled_ctrl_o = tl_s1n_24_ds_h2d[6];
  assign tl_s1n_24_ds_d2h[6] = tl_rgbled_ctrl_i;

  assign tl_hw_rev_o = tl_s1n_24_ds_h2d[7];
  assign tl_s1n_24_ds_d2h[7] = tl_hw_rev_i;

  assign tl_xadc_o = tl_s1n_24_ds_h2d[8];
  assign tl_s1n_24_ds_d2h[8] = tl_xadc_i;

  assign tl_timer_o = tl_s1n_24_ds_h2d[9];
  assign tl_s1n_24_ds_d2h[9] = tl_timer_i;

  assign tl_spi_board_o = tl_s1n_24_ds_h2d[10];
  assign tl_s1n_24_ds_d2h[10] = tl_spi_board_i;

  assign tl_spi_lcd_o = tl_s1n_24_ds_h2d[11];
  assign tl_s1n_24_ds_d2h[11] = tl_spi_lcd_i;

  assign tl_pwm0_o = tl_s1n_24_ds_h2d[12];
  assign tl_s1n_24_ds_d2h[12] = tl_pwm0_i;

  assign tl_uart0_o = tl_s1n_24_ds_h2d[13];
  assign tl_s1n_24_ds_d2h[13] = tl_uart0_i;

  assign tl_uart1_o = tl_s1n_24_ds_h2d[14];
  assign tl_s1n_24_ds_d2h[14] = tl_uart1_i;

  assign tl_uart2_o = tl_s1n_24_ds_h2d[15];
  assign tl_s1n_24_ds_d2h[15] = tl_uart2_i;

  assign tl_i2c0_o = tl_s1n_24_ds_h2d[16];
  assign tl_s1n_24_ds_d2h[16] = tl_i2c0_i;

  assign tl_i2c1_o = tl_s1n_24_ds_h2d[17];
  assign tl_s1n_24_ds_d2h[17] = tl_i2c1_i;

  assign tl_spi0_o = tl_s1n_24_ds_h2d[18];
  assign tl_s1n_24_ds_d2h[18] = tl_spi0_i;

  assign tl_spi1_o = tl_s1n_24_ds_h2d[19];
  assign tl_s1n_24_ds_d2h[19] = tl_spi1_i;

  assign tl_asf_26_us_h2d = tl_s1n_24_ds_h2d[20];
  assign tl_s1n_24_ds_d2h[20] = tl_asf_26_us_d2h;

  assign tl_rv_plic_o = tl_s1n_24_ds_h2d[21];
  assign tl_s1n_24_ds_d2h[21] = tl_rv_plic_i;

  assign tl_sm1_25_us_h2d[1] = tl_dbg_host_i;
  assign tl_dbg_host_o = tl_sm1_25_us_d2h[1];

  assign tl_s1n_24_us_h2d = tl_ibex_lsu_i;
  assign tl_ibex_lsu_o = tl_s1n_24_us_d2h;

  assign tl_sram_o = tl_sm1_25_ds_h2d;
  assign tl_sm1_25_ds_d2h = tl_sram_i;

  assign tl_usbdev_o = tl_asf_26_ds_h2d;
  assign tl_asf_26_ds_d2h = tl_usbdev_i;

  always_comb begin
    // default steering to generate error response if address is not within the range
    dev_sel_s1n_24 = 5'd22;
    if ((tl_s1n_24_us_h2d.a_address &
         ~(ADDR_MASK_SRAM)) == ADDR_SPACE_SRAM) begin
      dev_sel_s1n_24 = 5'd0;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_HYPERRAM)) == ADDR_SPACE_HYPERRAM) begin
      dev_sel_s1n_24 = 5'd1;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_REV_TAG)) == ADDR_SPACE_REV_TAG) begin
      dev_sel_s1n_24 = 5'd2;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_GPIO)) == ADDR_SPACE_GPIO) begin
      dev_sel_s1n_24 = 5'd3;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_PINMUX)) == ADDR_SPACE_PINMUX) begin
      dev_sel_s1n_24 = 5'd4;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_SYSTEM_INFO)) == ADDR_SPACE_SYSTEM_INFO) begin
      dev_sel_s1n_24 = 5'd5;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_RGBLED_CTRL)) == ADDR_SPACE_RGBLED_CTRL) begin
      dev_sel_s1n_24 = 5'd6;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_HW_REV)) == ADDR_SPACE_HW_REV) begin
      dev_sel_s1n_24 = 5'd7;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_XADC)) == ADDR_SPACE_XADC) begin
      dev_sel_s1n_24 = 5'd8;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_TIMER)) == ADDR_SPACE_TIMER) begin
      dev_sel_s1n_24 = 5'd9;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_BOARD)) == ADDR_SPACE_SPI_BOARD) begin
      dev_sel_s1n_24 = 5'd10;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_LCD)) == ADDR_SPACE_SPI_LCD) begin
      dev_sel_s1n_24 = 5'd11;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_PWM0)) == ADDR_SPACE_PWM0) begin
      dev_sel_s1n_24 = 5'd12;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_UART0)) == ADDR_SPACE_UART0) begin
      dev_sel_s1n_24 = 5'd13;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_UART1)) == ADDR_SPACE_UART1) begin
      dev_sel_s1n_24 = 5'd14;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_UART2)) == ADDR_SPACE_UART2) begin
      dev_sel_s1n_24 = 5'd15;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_I2C0)) == ADDR_SPACE_I2C0) begin
      dev_sel_s1n_24 = 5'd16;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_I2C1)) == ADDR_SPACE_I2C1) begin
      dev_sel_s1n_24 = 5'd17;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_SPI0)) == ADDR_SPACE_SPI0) begin
      dev_sel_s1n_24 = 5'd18;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_SPI1)) == ADDR_SPACE_SPI1) begin
      dev_sel_s1n_24 = 5'd19;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_USBDEV)) == ADDR_SPACE_USBDEV) begin
      dev_sel_s1n_24 = 5'd20;

    end else if ((tl_s1n_24_us_h2d.a_address &
                  ~(ADDR_MASK_RV_PLIC)) == ADDR_SPACE_RV_PLIC) begin
      dev_sel_s1n_24 = 5'd21;
end
  end


  // Instantiation phase
  tlul_socket_1n #(
    .HReqDepth (4'h0),
    .HRspDepth (4'h0),
    .DReqPass  (22'h1c0ea7),
    .DRspPass  (22'h1c0ea7),
    .DReqDepth (88'h1000111111000101011000),
    .DRspDepth (88'h1000111111000101011000),
    .N         (22)
  ) u_s1n_24 (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .tl_h_i       (tl_s1n_24_us_h2d),
    .tl_h_o       (tl_s1n_24_us_d2h),
    .tl_d_o       (tl_s1n_24_ds_h2d),
    .tl_d_i       (tl_s1n_24_ds_d2h),
    .dev_select_i (dev_sel_s1n_24)
  );
  tlul_socket_m1 #(
    .HReqPass  (2'h1),
    .HReqDepth (8'h10),
    .HRspDepth (8'h10),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_sm1_25 (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .tl_h_i       (tl_sm1_25_us_h2d),
    .tl_h_o       (tl_sm1_25_us_d2h),
    .tl_d_o       (tl_sm1_25_ds_h2d),
    .tl_d_i       (tl_sm1_25_ds_d2h)
  );
  tlul_fifo_async #(
    .ReqDepth        (1),
    .RspDepth        (1)
  ) u_asf_26 (
    .clk_h_i      (clk_sys_i),
    .rst_h_ni     (rst_sys_ni),
    .clk_d_i      (clk_usb_i),
    .rst_d_ni     (rst_usb_ni),
    .tl_h_i       (tl_asf_26_us_h2d),
    .tl_h_o       (tl_asf_26_us_d2h),
    .tl_d_o       (tl_asf_26_ds_h2d),
    .tl_d_i       (tl_asf_26_ds_d2h)
  );

endmodule
