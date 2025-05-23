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
  % for block in config.blocks:
  % if not block.name == "gpio":
  output tlul_pkg::tl_h2d_t tl_${block.name}_o[${block.name.upper()}_NUM],
  input  tlul_pkg::tl_d2h_t tl_${block.name}_i[${block.name.upper()}_NUM],
  % endif
  % endfor
  output tlul_pkg::tl_h2d_t tl_usbdev_o,
  input  tlul_pkg::tl_d2h_t tl_usbdev_i,
  output tlul_pkg::tl_h2d_t tl_dbg_dev_o,
  input  tlul_pkg::tl_d2h_t tl_dbg_dev_i,
  output tlul_pkg::tl_h2d_t tl_rv_plic_o,
  input  tlul_pkg::tl_d2h_t tl_rv_plic_i
);

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
    .tl_spi_lcd_i     (tl_spi_lcd_i),
    .tl_spi_lcd_o     (tl_spi_lcd_o),
    .tl_spi_ethmac_i  (tl_spi_ethmac_i),
    .tl_spi_ethmac_o  (tl_spi_ethmac_o),
    % for block in config.blocks:
    % if not block.name == "gpio":
    % for i in range(block.instances):
    .tl_${block.name}${i}_o        (tl_${block.name}_o[${i}]),
    .tl_${block.name}${i}_i        (tl_${block.name}_i[${i}]),
    % endfor
    % endif
    % endfor
    .tl_usbdev_o      (tl_usbdev_o),
    .tl_usbdev_i      (tl_usbdev_i),
    .tl_dbg_dev_o     (tl_dbg_dev_o),
    .tl_dbg_dev_i     (tl_dbg_dev_i),
    .tl_rv_plic_o     (tl_rv_plic_o),
    .tl_rv_plic_i     (tl_rv_plic_i),

    .scanmode_i       (prim_mubi_pkg::MuBi4False)
  );

endmodule
