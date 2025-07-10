// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module gpio_array #(
  parameter int unsigned GpiMaxWidth  =  8,
  parameter int unsigned GpoMaxWidth  = 16,
  parameter int unsigned AddrWidth    = 32,
  parameter int unsigned DataWidth    = 32,
  parameter int unsigned RegAddrWidth = 12,
  parameter int unsigned NumInstances =  1,
  parameter int unsigned GpiInstWidths[NumInstances] =  {8},
  parameter int unsigned GpoInstWidths[NumInstances] = {16}
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic                 device_req_i,
  input  logic [AddrWidth-1:0] device_addr_i,
  input  logic                 device_we_i,
  input  logic [3:0]           device_be_i,
  input  logic [DataWidth-1:0] device_wdata_i,
  output logic [DataWidth-1:0] device_rdata_o,

  input  logic [GpiMaxWidth-1:0] gp_i[NumInstances],
  output logic [GpoMaxWidth-1:0] gp_o[NumInstances],
  output logic [GpoMaxWidth-1:0] gp_o_en[NumInstances],

  output logic                   pcint_o[NumInstances]
);
  localparam int unsigned NumBytesPerInstance = 16 * DataWidth/8;
  localparam int unsigned AddrBitsPerInstance = $clog2(NumBytesPerInstance);
  localparam int unsigned AddrBitsInstanceIdx = $clog2(NumInstances);

  logic [DataWidth-1:0] device_rdata[NumInstances];
  logic [AddrBitsInstanceIdx-1:0] selected_inst_idx;

  assign selected_inst_idx = device_addr_i[RegAddrWidth-1:AddrBitsPerInstance];

  for (genvar i = 0; i < NumInstances; i++) begin
    gpio_core #(
      .GpiWidth  ( GpiInstWidths[i]    ),
      .GpoWidth  ( GpoInstWidths[i]    ),
      .AddrWidth ( AddrWidth           ),
      .DataWidth ( DataWidth           ),
      .RegAddr   ( AddrBitsPerInstance )
    ) u_gpio (
      .clk_i,
      .rst_ni,
      .device_req_i(selected_inst_idx == i ? device_req_i : 1'b0),
      .device_addr_i,
      .device_we_i,
      .device_be_i,
      .device_wdata_i,
      .device_rdata_o(device_rdata[i]),
      .gp_i(gp_i[i][GpiInstWidths[i]-1:0]),
      .gp_o(gp_o[i][GpoInstWidths[i]-1:0]),
      .gp_o_en(gp_o_en[i][GpoInstWidths[i]-1:0]),
      .pcint_o(pcint_o[i])
    );
  end

  // Read data is provided by gpio_core in the same cycle as the request,
  // so we can select rdata based on the request address.
  assign device_rdata_o = device_rdata[selected_inst_idx];

endmodule
