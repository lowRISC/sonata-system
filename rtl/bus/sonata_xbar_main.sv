// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// sonata_xbar_main module is a wrapper around the auto-generated xbar_main.
// rtl/bus/sonata_xbar_main.sv is automatically generated from rtl/templates/sonata_xbar_main.sv.tpl
// Please edit the template and run util/top_gen.py if you want to make changes.

module sonata_xbar_main
  import sonata_pkg::*;
(
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
  output tlul_pkg::tl_h2d_t tl_system_info_o,
  input  tlul_pkg::tl_d2h_t tl_system_info_i,
  output tlul_pkg::tl_h2d_t tl_rgbled_ctrl_o,
  input  tlul_pkg::tl_d2h_t tl_rgbled_ctrl_i,
  output tlul_pkg::tl_h2d_t tl_hw_rev_o,
  input  tlul_pkg::tl_d2h_t tl_hw_rev_i,
  output tlul_pkg::tl_h2d_t tl_xadc_o,
  input  tlul_pkg::tl_d2h_t tl_xadc_i,
  output tlul_pkg::tl_h2d_t tl_timer_o,
  input  tlul_pkg::tl_d2h_t tl_timer_i,
  output tlul_pkg::tl_h2d_t tl_spi_lcd_o,
  input  tlul_pkg::tl_d2h_t tl_spi_lcd_i,
  output tlul_pkg::tl_h2d_t tl_spi_ethmac_o,
  input  tlul_pkg::tl_d2h_t tl_spi_ethmac_i,
  output tlul_pkg::tl_h2d_t tl_pwm_o[PWM_NUM],
  input  tlul_pkg::tl_d2h_t tl_pwm_i[PWM_NUM],
  output tlul_pkg::tl_h2d_t tl_uart_o[UART_NUM],
  input  tlul_pkg::tl_d2h_t tl_uart_i[UART_NUM],
  output tlul_pkg::tl_h2d_t tl_i2c_o[I2C_NUM],
  input  tlul_pkg::tl_d2h_t tl_i2c_i[I2C_NUM],
  output tlul_pkg::tl_h2d_t tl_spi_o[SPI_NUM],
  input  tlul_pkg::tl_d2h_t tl_spi_i[SPI_NUM],
  output tlul_pkg::tl_h2d_t tl_usbdev_o,
  input  tlul_pkg::tl_d2h_t tl_usbdev_i,
  output tlul_pkg::tl_h2d_t tl_dbg_dev_o,
  input  tlul_pkg::tl_d2h_t tl_dbg_dev_i,
  output tlul_pkg::tl_h2d_t tl_rv_plic_o,
  input  tlul_pkg::tl_d2h_t tl_rv_plic_i
);

  // Inter-xbar interfaces
  tlul_pkg::tl_h2d_t tl_main_to_peri;
  tlul_pkg::tl_d2h_t tl_peri_to_main;

  xbar_main xbar (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),
    .clk_usb_i        (clk_usb_i),
    .rst_usb_ni       (rst_usb_ni),

    // Host interfaces.
    .tl_ibex_lsu_i    (tl_ibex_lsu_i),
    .tl_ibex_lsu_o    (tl_ibex_lsu_o),
    .tl_dbg_host_i    (tl_dbg_host_i),
    .tl_dbg_host_o    (tl_dbg_host_o),

    // Device interfaces.
    .tl_sram_o        (tl_sram_o),
    .tl_sram_i        (tl_sram_i),
    .tl_hyperram_o    (tl_hyperram_o),
    .tl_hyperram_i    (tl_hyperram_i),
    .tl_rev_tag_o     (tl_rev_tag_o),
    .tl_rev_tag_i     (tl_rev_tag_i),
    .tl_peri_o        (tl_main_to_peri),
    .tl_peri_i        (tl_peri_to_main),
    .tl_spi_lcd_i     (tl_spi_lcd_i),
    .tl_spi_lcd_o     (tl_spi_lcd_o),
    .tl_spi_ethmac_i  (tl_spi_ethmac_i),
    .tl_spi_ethmac_o  (tl_spi_ethmac_o),
    .tl_i2c0_o        (tl_i2c_o[0]),
    .tl_i2c0_i        (tl_i2c_i[0]),
    .tl_i2c1_o        (tl_i2c_o[1]),
    .tl_i2c1_i        (tl_i2c_i[1]),
    .tl_spi0_o        (tl_spi_o[0]),
    .tl_spi0_i        (tl_spi_i[0]),
    .tl_spi1_o        (tl_spi_o[1]),
    .tl_spi1_i        (tl_spi_i[1]),
    .tl_spi2_o        (tl_spi_o[2]),
    .tl_spi2_i        (tl_spi_i[2]),
    .tl_usbdev_o      (tl_usbdev_o),
    .tl_usbdev_i      (tl_usbdev_i),
    .tl_dbg_dev_o     (tl_dbg_dev_o),
    .tl_dbg_dev_i     (tl_dbg_dev_i),
    .tl_rv_plic_o     (tl_rv_plic_o),
    .tl_rv_plic_i     (tl_rv_plic_i),

    .scanmode_i       (prim_mubi_pkg::MuBi4False)
  );

  xbar_peri xbar2 (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),

    // Host interfaces.
    .tl_main_i    (tl_main_to_peri),
    .tl_main_o    (tl_peri_to_main),

    // Device interfaces.
    .tl_gpio_o        (tl_gpio_o),
    .tl_gpio_i        (tl_gpio_i),
    .tl_pinmux_o      (tl_pinmux_o),
    .tl_pinmux_i      (tl_pinmux_i),
    .tl_system_info_o (tl_system_info_o),
    .tl_system_info_i (tl_system_info_i),
    .tl_rgbled_ctrl_o (tl_rgbled_ctrl_o),
    .tl_rgbled_ctrl_i (tl_rgbled_ctrl_i),
    .tl_hw_rev_o      (tl_hw_rev_o),
    .tl_hw_rev_i      (tl_hw_rev_i),
    .tl_xadc_o        (tl_xadc_o),
    .tl_xadc_i        (tl_xadc_i),
    .tl_timer_o       (tl_timer_o),
    .tl_timer_i       (tl_timer_i),
    .tl_pwm0_o        (tl_pwm_o[0]),
    .tl_pwm0_i        (tl_pwm_i[0]),
    .tl_uart0_o        (tl_uart_o[0]),
    .tl_uart0_i        (tl_uart_i[0]),
    .tl_uart1_o        (tl_uart_o[1]),
    .tl_uart1_i        (tl_uart_i[1]),
    .tl_uart2_o        (tl_uart_o[2]),
    .tl_uart2_i        (tl_uart_i[2]),

    .scanmode_i       (prim_mubi_pkg::MuBi4False)
  );

endmodule
