// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// xbar_main module generated by `tlgen.py` tool
// all reset signals should be generated from one reset signal to not make any deadlock
//
// Interconnect
// ibex_lsu
//   -> s1n_30
//     -> sm1_31
//       -> sram
//     -> hyperram
//     -> rev_tag
//     -> gpio
//     -> pwm
//     -> rpi_gpio
//     -> ard_gpio
//     -> pmod_gpio
//     -> rgbled_ctrl
//     -> hw_rev
//     -> xadc
//     -> timer
//     -> uart0
//     -> uart1
//     -> uart2
//     -> uart3
//     -> uart4
//     -> i2c0
//     -> i2c1
//     -> spi_flash
//     -> spi_lcd
//     -> spi_eth
//     -> spi_rp0
//     -> spi_rp1
//     -> spi_ard
//     -> spi_mkr
//     -> asf_32
//       -> usbdev
//     -> rv_plic
// dbg_host
//   -> sm1_31
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
  output tlul_pkg::tl_h2d_t tl_pwm_o,
  input  tlul_pkg::tl_d2h_t tl_pwm_i,
  output tlul_pkg::tl_h2d_t tl_rpi_gpio_o,
  input  tlul_pkg::tl_d2h_t tl_rpi_gpio_i,
  output tlul_pkg::tl_h2d_t tl_ard_gpio_o,
  input  tlul_pkg::tl_d2h_t tl_ard_gpio_i,
  output tlul_pkg::tl_h2d_t tl_pmod_gpio_o,
  input  tlul_pkg::tl_d2h_t tl_pmod_gpio_i,
  output tlul_pkg::tl_h2d_t tl_rgbled_ctrl_o,
  input  tlul_pkg::tl_d2h_t tl_rgbled_ctrl_i,
  output tlul_pkg::tl_h2d_t tl_hw_rev_o,
  input  tlul_pkg::tl_d2h_t tl_hw_rev_i,
  output tlul_pkg::tl_h2d_t tl_xadc_o,
  input  tlul_pkg::tl_d2h_t tl_xadc_i,
  output tlul_pkg::tl_h2d_t tl_timer_o,
  input  tlul_pkg::tl_d2h_t tl_timer_i,
  output tlul_pkg::tl_h2d_t tl_uart0_o,
  input  tlul_pkg::tl_d2h_t tl_uart0_i,
  output tlul_pkg::tl_h2d_t tl_uart1_o,
  input  tlul_pkg::tl_d2h_t tl_uart1_i,
  output tlul_pkg::tl_h2d_t tl_uart2_o,
  input  tlul_pkg::tl_d2h_t tl_uart2_i,
  output tlul_pkg::tl_h2d_t tl_uart3_o,
  input  tlul_pkg::tl_d2h_t tl_uart3_i,
  output tlul_pkg::tl_h2d_t tl_uart4_o,
  input  tlul_pkg::tl_d2h_t tl_uart4_i,
  output tlul_pkg::tl_h2d_t tl_i2c0_o,
  input  tlul_pkg::tl_d2h_t tl_i2c0_i,
  output tlul_pkg::tl_h2d_t tl_i2c1_o,
  input  tlul_pkg::tl_d2h_t tl_i2c1_i,
  output tlul_pkg::tl_h2d_t tl_spi_flash_o,
  input  tlul_pkg::tl_d2h_t tl_spi_flash_i,
  output tlul_pkg::tl_h2d_t tl_spi_lcd_o,
  input  tlul_pkg::tl_d2h_t tl_spi_lcd_i,
  output tlul_pkg::tl_h2d_t tl_spi_eth_o,
  input  tlul_pkg::tl_d2h_t tl_spi_eth_i,
  output tlul_pkg::tl_h2d_t tl_spi_rp0_o,
  input  tlul_pkg::tl_d2h_t tl_spi_rp0_i,
  output tlul_pkg::tl_h2d_t tl_spi_rp1_o,
  input  tlul_pkg::tl_d2h_t tl_spi_rp1_i,
  output tlul_pkg::tl_h2d_t tl_spi_ard_o,
  input  tlul_pkg::tl_d2h_t tl_spi_ard_i,
  output tlul_pkg::tl_h2d_t tl_spi_mkr_o,
  input  tlul_pkg::tl_d2h_t tl_spi_mkr_i,
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

  tl_h2d_t tl_s1n_30_us_h2d ;
  tl_d2h_t tl_s1n_30_us_d2h ;


  tl_h2d_t tl_s1n_30_ds_h2d [28];
  tl_d2h_t tl_s1n_30_ds_d2h [28];

  // Create steering signal
  logic [4:0] dev_sel_s1n_30;


  tl_h2d_t tl_sm1_31_us_h2d [2];
  tl_d2h_t tl_sm1_31_us_d2h [2];

  tl_h2d_t tl_sm1_31_ds_h2d ;
  tl_d2h_t tl_sm1_31_ds_d2h ;

  tl_h2d_t tl_asf_32_us_h2d ;
  tl_d2h_t tl_asf_32_us_d2h ;
  tl_h2d_t tl_asf_32_ds_h2d ;
  tl_d2h_t tl_asf_32_ds_d2h ;



  assign tl_sm1_31_us_h2d[0] = tl_s1n_30_ds_h2d[0];
  assign tl_s1n_30_ds_d2h[0] = tl_sm1_31_us_d2h[0];

  assign tl_hyperram_o = tl_s1n_30_ds_h2d[1];
  assign tl_s1n_30_ds_d2h[1] = tl_hyperram_i;

  assign tl_rev_tag_o = tl_s1n_30_ds_h2d[2];
  assign tl_s1n_30_ds_d2h[2] = tl_rev_tag_i;

  assign tl_gpio_o = tl_s1n_30_ds_h2d[3];
  assign tl_s1n_30_ds_d2h[3] = tl_gpio_i;

  assign tl_pwm_o = tl_s1n_30_ds_h2d[4];
  assign tl_s1n_30_ds_d2h[4] = tl_pwm_i;

  assign tl_rpi_gpio_o = tl_s1n_30_ds_h2d[5];
  assign tl_s1n_30_ds_d2h[5] = tl_rpi_gpio_i;

  assign tl_ard_gpio_o = tl_s1n_30_ds_h2d[6];
  assign tl_s1n_30_ds_d2h[6] = tl_ard_gpio_i;

  assign tl_pmod_gpio_o = tl_s1n_30_ds_h2d[7];
  assign tl_s1n_30_ds_d2h[7] = tl_pmod_gpio_i;

  assign tl_rgbled_ctrl_o = tl_s1n_30_ds_h2d[8];
  assign tl_s1n_30_ds_d2h[8] = tl_rgbled_ctrl_i;

  assign tl_hw_rev_o = tl_s1n_30_ds_h2d[9];
  assign tl_s1n_30_ds_d2h[9] = tl_hw_rev_i;

  assign tl_xadc_o = tl_s1n_30_ds_h2d[10];
  assign tl_s1n_30_ds_d2h[10] = tl_xadc_i;

  assign tl_timer_o = tl_s1n_30_ds_h2d[11];
  assign tl_s1n_30_ds_d2h[11] = tl_timer_i;

  assign tl_uart0_o = tl_s1n_30_ds_h2d[12];
  assign tl_s1n_30_ds_d2h[12] = tl_uart0_i;

  assign tl_uart1_o = tl_s1n_30_ds_h2d[13];
  assign tl_s1n_30_ds_d2h[13] = tl_uart1_i;

  assign tl_uart2_o = tl_s1n_30_ds_h2d[14];
  assign tl_s1n_30_ds_d2h[14] = tl_uart2_i;

  assign tl_uart3_o = tl_s1n_30_ds_h2d[15];
  assign tl_s1n_30_ds_d2h[15] = tl_uart3_i;

  assign tl_uart4_o = tl_s1n_30_ds_h2d[16];
  assign tl_s1n_30_ds_d2h[16] = tl_uart4_i;

  assign tl_i2c0_o = tl_s1n_30_ds_h2d[17];
  assign tl_s1n_30_ds_d2h[17] = tl_i2c0_i;

  assign tl_i2c1_o = tl_s1n_30_ds_h2d[18];
  assign tl_s1n_30_ds_d2h[18] = tl_i2c1_i;

  assign tl_spi_flash_o = tl_s1n_30_ds_h2d[19];
  assign tl_s1n_30_ds_d2h[19] = tl_spi_flash_i;

  assign tl_spi_lcd_o = tl_s1n_30_ds_h2d[20];
  assign tl_s1n_30_ds_d2h[20] = tl_spi_lcd_i;

  assign tl_spi_eth_o = tl_s1n_30_ds_h2d[21];
  assign tl_s1n_30_ds_d2h[21] = tl_spi_eth_i;

  assign tl_spi_rp0_o = tl_s1n_30_ds_h2d[22];
  assign tl_s1n_30_ds_d2h[22] = tl_spi_rp0_i;

  assign tl_spi_rp1_o = tl_s1n_30_ds_h2d[23];
  assign tl_s1n_30_ds_d2h[23] = tl_spi_rp1_i;

  assign tl_spi_ard_o = tl_s1n_30_ds_h2d[24];
  assign tl_s1n_30_ds_d2h[24] = tl_spi_ard_i;

  assign tl_spi_mkr_o = tl_s1n_30_ds_h2d[25];
  assign tl_s1n_30_ds_d2h[25] = tl_spi_mkr_i;

  assign tl_asf_32_us_h2d = tl_s1n_30_ds_h2d[26];
  assign tl_s1n_30_ds_d2h[26] = tl_asf_32_us_d2h;

  assign tl_rv_plic_o = tl_s1n_30_ds_h2d[27];
  assign tl_s1n_30_ds_d2h[27] = tl_rv_plic_i;

  assign tl_sm1_31_us_h2d[1] = tl_dbg_host_i;
  assign tl_dbg_host_o = tl_sm1_31_us_d2h[1];

  assign tl_s1n_30_us_h2d = tl_ibex_lsu_i;
  assign tl_ibex_lsu_o = tl_s1n_30_us_d2h;

  assign tl_sram_o = tl_sm1_31_ds_h2d;
  assign tl_sm1_31_ds_d2h = tl_sram_i;

  assign tl_usbdev_o = tl_asf_32_ds_h2d;
  assign tl_asf_32_ds_d2h = tl_usbdev_i;

  always_comb begin
    // default steering to generate error response if address is not within the range
    dev_sel_s1n_30 = 5'd28;
    if ((tl_s1n_30_us_h2d.a_address &
         ~(ADDR_MASK_SRAM)) == ADDR_SPACE_SRAM) begin
      dev_sel_s1n_30 = 5'd0;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_HYPERRAM)) == ADDR_SPACE_HYPERRAM) begin
      dev_sel_s1n_30 = 5'd1;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_REV_TAG)) == ADDR_SPACE_REV_TAG) begin
      dev_sel_s1n_30 = 5'd2;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_GPIO)) == ADDR_SPACE_GPIO) begin
      dev_sel_s1n_30 = 5'd3;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_PWM)) == ADDR_SPACE_PWM) begin
      dev_sel_s1n_30 = 5'd4;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_RPI_GPIO)) == ADDR_SPACE_RPI_GPIO) begin
      dev_sel_s1n_30 = 5'd5;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_ARD_GPIO)) == ADDR_SPACE_ARD_GPIO) begin
      dev_sel_s1n_30 = 5'd6;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_PMOD_GPIO)) == ADDR_SPACE_PMOD_GPIO) begin
      dev_sel_s1n_30 = 5'd7;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_RGBLED_CTRL)) == ADDR_SPACE_RGBLED_CTRL) begin
      dev_sel_s1n_30 = 5'd8;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_HW_REV)) == ADDR_SPACE_HW_REV) begin
      dev_sel_s1n_30 = 5'd9;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_XADC)) == ADDR_SPACE_XADC) begin
      dev_sel_s1n_30 = 5'd10;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_TIMER)) == ADDR_SPACE_TIMER) begin
      dev_sel_s1n_30 = 5'd11;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_UART0)) == ADDR_SPACE_UART0) begin
      dev_sel_s1n_30 = 5'd12;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_UART1)) == ADDR_SPACE_UART1) begin
      dev_sel_s1n_30 = 5'd13;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_UART2)) == ADDR_SPACE_UART2) begin
      dev_sel_s1n_30 = 5'd14;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_UART3)) == ADDR_SPACE_UART3) begin
      dev_sel_s1n_30 = 5'd15;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_UART4)) == ADDR_SPACE_UART4) begin
      dev_sel_s1n_30 = 5'd16;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_I2C0)) == ADDR_SPACE_I2C0) begin
      dev_sel_s1n_30 = 5'd17;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_I2C1)) == ADDR_SPACE_I2C1) begin
      dev_sel_s1n_30 = 5'd18;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_FLASH)) == ADDR_SPACE_SPI_FLASH) begin
      dev_sel_s1n_30 = 5'd19;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_LCD)) == ADDR_SPACE_SPI_LCD) begin
      dev_sel_s1n_30 = 5'd20;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_ETH)) == ADDR_SPACE_SPI_ETH) begin
      dev_sel_s1n_30 = 5'd21;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_RP0)) == ADDR_SPACE_SPI_RP0) begin
      dev_sel_s1n_30 = 5'd22;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_RP1)) == ADDR_SPACE_SPI_RP1) begin
      dev_sel_s1n_30 = 5'd23;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_ARD)) == ADDR_SPACE_SPI_ARD) begin
      dev_sel_s1n_30 = 5'd24;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_SPI_MKR)) == ADDR_SPACE_SPI_MKR) begin
      dev_sel_s1n_30 = 5'd25;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_USBDEV)) == ADDR_SPACE_USBDEV) begin
      dev_sel_s1n_30 = 5'd26;

    end else if ((tl_s1n_30_us_h2d.a_address &
                  ~(ADDR_MASK_RV_PLIC)) == ADDR_SPACE_RV_PLIC) begin
      dev_sel_s1n_30 = 5'd27;
end
  end


  // Instantiation phase
  tlul_socket_1n #(
    .HReqDepth (4'h0),
    .HRspDepth (4'h0),
    .DReqDepth (112'h0),
    .DRspDepth (112'h0),
    .N         (28)
  ) u_s1n_30 (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .tl_h_i       (tl_s1n_30_us_h2d),
    .tl_h_o       (tl_s1n_30_us_d2h),
    .tl_d_o       (tl_s1n_30_ds_h2d),
    .tl_d_i       (tl_s1n_30_ds_d2h),
    .dev_select_i (dev_sel_s1n_30)
  );
  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_sm1_31 (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .tl_h_i       (tl_sm1_31_us_h2d),
    .tl_h_o       (tl_sm1_31_us_d2h),
    .tl_d_o       (tl_sm1_31_ds_h2d),
    .tl_d_i       (tl_sm1_31_ds_d2h)
  );
  tlul_fifo_async #(
    .ReqDepth        (1),
    .RspDepth        (1)
  ) u_asf_32 (
    .clk_h_i      (clk_sys_i),
    .rst_h_ni     (rst_sys_ni),
    .clk_d_i      (clk_usb_i),
    .rst_d_ni     (rst_usb_ni),
    .tl_h_i       (tl_asf_32_us_h2d),
    .tl_h_o       (tl_asf_32_us_d2h),
    .tl_d_o       (tl_asf_32_ds_h2d),
    .tl_d_i       (tl_asf_32_ds_d2h)
  );

endmodule
