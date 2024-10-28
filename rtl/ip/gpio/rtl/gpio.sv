// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module gpio #(
  parameter int unsigned GpiWidth     =  8,
  parameter int unsigned GpoWidth     = 16,
  parameter int unsigned AddrWidth    = 32,
  parameter int unsigned DataWidth    = 32,
  parameter int unsigned RegAddr      = 12,
  parameter int unsigned NumInstances =  1
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  input  logic [GpiWidth-1:0] gp_i[NumInstances],
  output logic [GpoWidth-1:0] gp_o[NumInstances],
  output logic [GpoWidth-1:0] gp_o_en[NumInstances]
);
  localparam int unsigned RegAddrWidth  = 8;

  logic                 device_req;
  logic [AddrWidth-1:0] device_addr;
  logic                 device_re; // Read enable.
  logic                 device_we; // Write enable.
  logic [3:0]           device_be;
  logic [DataWidth-1:0] device_wdata;
  logic [DataWidth-1:0] device_rdata;

  assign device_req = device_re | device_we;

  // Tie off upper bits of address.
  assign device_addr[AddrWidth-1:RegAddrWidth] = '0;

  gpio_array #(
    .GpiWidth     ( GpiWidth     ),
    .GpoWidth     ( GpoWidth     ),
    .AddrWidth    ( AddrWidth    ),
    .DataWidth    ( DataWidth    ),
    .RegAddr      ( RegAddr      ),
    .NumInstances ( NumInstances )
  ) u_gpio (
    .clk_i,
    .rst_ni,

    .device_req_i    (device_req),
    .device_addr_i   (device_addr),
    .device_we_i     (device_we),
    .device_be_i     (device_be),
    .device_wdata_i  (device_wdata),
    .device_rvalid_o (),
    .device_rdata_o  (device_rdata),

    .gp_i,
    .gp_o,
    .gp_o_en
  );

  tlul_adapter_reg #(
    .AccessLatency    ( 1 )
  ) gpio_device_adapter (
    .clk_i,
    .rst_ni,

    // TL-UL interface.
    .tl_i,
    .tl_o,

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re),
    .we_o         (device_we),
    .addr_o       (device_addr[RegAddrWidth-1:0]),
    .wdata_o      (device_wdata),
    .be_o         (device_be),
    .busy_i       (1'b0),
    .rdata_i      (device_rdata),
    .error_i      (1'b0)
  );
endmodule
