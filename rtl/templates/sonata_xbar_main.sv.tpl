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
  % for num, block in enumerate(config.blocks):
  output tlul_pkg::tl_h2d_t tl_${block.name}_o${("[" + block.name.upper()+"_NUM]") if block.instances > 1 else ""},
  input  tlul_pkg::tl_d2h_t tl_${block.name}_i${("[" + block.name.upper()+"_NUM]") if block.instances > 1 else ""}${',' if num < len(config.blocks) - 1 else ''}
  % endfor
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
    % for block in config.blocks:
      % for i in range(block.instances):
    .tl_${block.name}${i if block.instances > 1 else ""}_o        (tl_${block.name}_o${f"[{i}]" if block.instances > 1 else ""}),
    .tl_${block.name}${i if block.instances > 1 else ""}_i        (tl_${block.name}_i${f"[{i}]" if block.instances > 1 else ""}),
      % endfor
    % endfor

    .scanmode_i       (prim_mubi_pkg::MuBi4False)
  );

endmodule
