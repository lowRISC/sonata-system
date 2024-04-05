// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module sram #(
  // (Byte-addressable) address width of the SRAM.
  parameter int unsigned AddrWidth = 17,
  parameter InitFile               = ""
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  tlul_pkg::tl_h2d_t tl_a_i,
    output tlul_pkg::tl_d2h_t tl_a_o,

    input  tlul_pkg::tl_h2d_t tl_b_i,
    output tlul_pkg::tl_d2h_t tl_b_o
  );

  localparam int unsigned BusAddrWidth  = 32;
  localparam int unsigned BusByteEnable = 4;
  localparam int unsigned BusDataWidth  = 32;

  logic                     mem_a_req;
  logic [BusAddrWidth-1:0]  mem_a_addr;
  logic                     mem_a_we;
  logic [BusByteEnable-1:0] mem_a_be;
  logic [BusDataWidth-1:0]  mem_a_wdata;
  logic                     mem_a_wcap;
  logic                     mem_a_rvalid;
  logic [BusDataWidth-1:0]  mem_a_rdata;
  logic                     mem_a_rcap;
  logic                     mem_a_err;

  logic                    mem_b_req;
  logic                    mem_b_rvalid;
  logic [BusAddrWidth-1:0] mem_b_addr;
  logic [BusDataWidth-1:0] mem_b_rdata;
  logic                    unused_mem_b_rcap;

  assign mem_a_err  = 1'b0;

  // TL-UL device adapters

  tlul_adapter_sram #(
    .SramAw           ( AddrWidth - 2 ),
    .EnableRspIntgGen ( 1             )
  ) sram_b_device_adapter (
    .clk_i,
    .rst_ni,

    // TL-UL interface.
    .tl_i(tl_b_i),
    .tl_o(tl_b_o),

    // Control interface.
    .en_ifetch_i (prim_mubi_pkg::MuBi4True),

    // SRAM interface.
    .req_o(mem_b_req),
    .req_type_o(),
    .gnt_i(mem_b_req),
    .we_o(),
    .addr_o(mem_b_addr[AddrWidth-1:2]),
    .wdata_o(),
    .wdata_cap_o(),
    .wmask_o(),
    .intg_error_o(),
    .rdata_i(mem_b_rdata),
    .rdata_cap_i(1'b0),
    .rvalid_i(mem_b_rvalid),
    .rerror_i(2'b00)
  );

  assign mem_b_addr[BusAddrWidth-1:AddrWidth] = '0;
  assign mem_b_addr[1:0] = '0;

  logic [BusDataWidth-1:0] sram_data_bit_enable;
  logic [1:0]              sram_data_read_error;

  tlul_adapter_sram #(
    .SramAw           ( AddrWidth - 2 ),
    .EnableRspIntgGen ( 1             )
  ) sram_a_device_adapter (
    .clk_i,
    .rst_ni,

    // TL-UL interface.
    .tl_i(tl_a_i),
    .tl_o(tl_a_o),

    // Control interface.
    .en_ifetch_i(prim_mubi_pkg::MuBi4False),

    // SRAM interface.
    .req_o       (mem_a_req),
    .req_type_o  (),
    .gnt_i       (mem_a_req),
    .we_o        (mem_a_we),
    .addr_o      (mem_a_addr[AddrWidth-1:2]),
    .wdata_o     (mem_a_wdata),
    .wdata_cap_o (mem_a_wcap),
    .wmask_o     (sram_data_bit_enable),
    .intg_error_o(),
    .rdata_i     (mem_a_rdata),
    .rdata_cap_i (mem_a_rcap),
    .rvalid_i    (mem_a_rvalid),
    .rerror_i    (sram_data_read_error)
  );

  // Tie off upper and lower bits of address.
  assign mem_a_addr[BusAddrWidth-1:AddrWidth] = '0;
  assign mem_a_addr[1:0] = '0;

  // Translate bit-level enable signals to Byte-level.
  assign mem_a_be[0] = |sram_data_bit_enable[ 7: 0];
  assign mem_a_be[1] = |sram_data_bit_enable[15: 8];
  assign mem_a_be[2] = |sram_data_bit_enable[23:16];
  assign mem_a_be[3] = |sram_data_bit_enable[31:24];

  // Internal to the TLUL SRAM adapter, 2'b10 is an uncorrectable error and 2'b00 is no error.
  // The following line converts the single bit error into this two bit format:
  assign sram_data_read_error = {mem_a_err, 1'b0};

  localparam int RamDepth = 2 ** (AddrWidth - 2);

  ram_2p #(
    .Depth       ( RamDepth ),
    .MemInitFile ( InitFile )
  ) u_ram (
    .clk_i,
    .rst_ni,

    .a_req_i   (mem_a_req),
    .a_we_i    (mem_a_we),
    .a_be_i    (mem_a_be),
    .a_addr_i  (mem_a_addr),
    .a_wdata_i (mem_a_wdata),
    .a_rvalid_o(mem_a_rvalid),
    .a_rdata_o (mem_a_rdata),

    .b_req_i   (mem_b_req),
    .b_we_i    (1'b0),
    .b_be_i    (BusByteEnable'(0)),
    .b_addr_i  (mem_b_addr),
    .b_wdata_i (BusDataWidth'(0)),
    .b_rvalid_o(mem_b_rvalid),
    .b_rdata_o (mem_b_rdata)
  );

  prim_ram_2p #(
    .Width ( 1        ),
    .Depth ( RamDepth )
  ) u_cap_ram (
    .clk_a_i   (clk_i),
    .clk_b_i   (rst_ni),
    .cfg_i     ('0),
    .a_req_i   (mem_a_req),
    .a_write_i (&mem_a_we),
    .a_addr_i  (mem_a_addr[AddrWidth-1:2]),
    .a_wdata_i (mem_a_wcap),
    .a_wmask_i (&mem_a_we),
    .a_rdata_o (mem_a_rcap),
    .b_req_i   (mem_b_req),
    .b_write_i (1'b0),
    .b_wmask_i (1'b0),
    .b_addr_i  (mem_b_addr[AddrWidth-1:2]),
    .b_wdata_i (1'b0),
    .b_rdata_o (unused_mem_b_rcap)
  );

endmodule
