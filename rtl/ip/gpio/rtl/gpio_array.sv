// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module gpio_array #(
  parameter int unsigned GpiWidth     =  8,
  parameter int unsigned GpoWidth     = 16,
  parameter int unsigned AddrWidth    = 32,
  parameter int unsigned DataWidth    = 32,
  parameter int unsigned RegAddrWidth = 12,
  parameter int unsigned NumInstances =  1
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic                 device_req_i,
  input  logic [AddrWidth-1:0] device_addr_i,
  input  logic                 device_we_i,
  input  logic [3:0]           device_be_i,
  input  logic [DataWidth-1:0] device_wdata_i,
  output logic                 device_rvalid_o,
  output logic [DataWidth-1:0] device_rdata_o,

  input  logic [GpiWidth-1:0] gp_i[NumInstances],
  output logic [GpoWidth-1:0] gp_o[NumInstances],
  output logic [GpoWidth-1:0] gp_o_en[NumInstances]
);
  localparam int unsigned NumBytesPerInstance = 16 * DataWidth/8;
  localparam int unsigned AddrBitsPerInstance = $clog2(NumBytesPerInstance);

  logic device_read_valids[NumInstances];
  logic [DataWidth-1:0] device_read_datas[NumInstances];

  for (genvar i = 0; i < NumInstances; i++) begin
    logic device_selector;
    assign device_selector = (device_addr_i[RegAddrWidth-1:AddrBitsPerInstance]== i) ?
                             device_req_i : 1'b0;

    gpio_core #(
      .GpiWidth  ( GpiWidth            ),
      .GpoWidth  ( GpoWidth            ),
      .AddrWidth ( AddrWidth           ),
      .DataWidth ( DataWidth           ),
      .RegAddr   ( AddrBitsPerInstance )
    ) u_gpio (
      .clk_i,
      .rst_ni,
      .device_req_i(device_selector),
      .device_addr_i,
      .device_we_i,
      .device_be_i,
      .device_wdata_i,
      .device_rvalid_o(device_read_valids[i]),
      .device_rdata_o(device_read_datas[i]),
      .gp_i(gp_i[i]),
      .gp_o(gp_o[i]),
      .gp_o_en(gp_o_en[i])
    );
  end

  always_comb begin
    device_rvalid_o = 1'b0;
    device_rdata_o = {DataWidth{1'b0}};
    for (integer i = 0; i < NumInstances; i++) begin
      if (device_read_valids[i] == 1'b1) begin
        device_rvalid_o = device_read_valids[i];
        device_rdata_o = device_read_datas[i];
      end
    end
  end

endmodule
