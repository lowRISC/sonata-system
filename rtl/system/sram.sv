// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module sram #(
  // (Byte-addressable) address width of the SRAM.
  parameter int unsigned AddrWidth = 17,
  // Data width of the SRAM.
  parameter int unsigned DataWidth = 32,
  // Grouping of data bits into sub-words.
  parameter int unsigned DataBitsPerMask = 8,
  parameter InitFile               = ""
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  tlul_pkg::tl_h2d_t tl_a_i,
    output tlul_pkg::tl_d2h_t tl_a_o,

    input  tlul_pkg::tl_h2d_t tl_b_i,
    output tlul_pkg::tl_d2h_t tl_b_o
  );

  // Bit offset of word address.
  localparam int unsigned AOff = $clog2(DataWidth / 8);
  // Number of address bits to select a word from the SRAM.
  localparam int unsigned SramAw = AddrWidth - AOff;

  logic                     mem_a_req;
  logic [AddrWidth-1:AOff]  mem_a_addr;
  logic                     mem_a_we;
  logic [DataWidth-1:0]     mem_a_wmask;

  logic [DataWidth-1:0]     mem_a_wdata;
  logic                     mem_a_wcap;
  logic                     mem_a_rvalid;
  logic [DataWidth-1:0]     mem_a_rdata;
  logic                     mem_a_rcap;

  logic                    mem_b_req;
  logic                    mem_b_we;
  logic                    mem_b_rvalid;
  logic [AddrWidth-1:AOff] mem_b_addr;
  logic [DataWidth-1:0]    mem_b_rdata;
  logic                    unused_mem_b_rcap;

  // TL-UL device adapters
  tlul_adapter_sram #(
    .SramAw           ( SramAw ),
    .EnableRspIntgGen ( 1      )
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
    .addr_o      (mem_a_addr),
    .wdata_o     (mem_a_wdata),
    .wdata_cap_o (mem_a_wcap),
    .wmask_o     (mem_a_wmask),
    .intg_error_o(),
    .rdata_i     (mem_a_rdata),
    .rdata_cap_i (mem_a_rcap),
    .rvalid_i    (mem_a_rvalid),
    .rerror_i    (2'b00)
  );

  tlul_adapter_sram #(
    .SramAw           ( SramAw ),
    .EnableRspIntgGen ( 1      )
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
    .we_o(mem_b_we),
    .addr_o(mem_b_addr),
    .wdata_o(),
    .wdata_cap_o(),
    .wmask_o(),
    .intg_error_o(),
    .rdata_i(mem_b_rdata),
    .rdata_cap_i(1'b0),
    .rvalid_i(mem_b_rvalid),
    .rerror_i(2'b00)
  );

  // Number of words in data memory.
  localparam int unsigned RamDepth = 2 ** SramAw;

  // Instantiate RAM blocks

  // Data memory
  prim_ram_2p #(
    .Width           ( DataWidth       ),
    .DataBitsPerMask ( DataBitsPerMask ),
    .Depth           ( RamDepth        ),
    .MemInitFile     ( InitFile        )
  ) u_ram (
    .clk_a_i   (clk_i),
    .clk_b_i   (clk_i),

    .a_req_i   (mem_a_req),
    .a_write_i (mem_a_we),
    .a_addr_i  (mem_a_addr),
    .a_wdata_i (mem_a_wdata),
    .a_wmask_i (mem_a_wmask),
    .a_rdata_o (mem_a_rdata),

    .b_req_i   (mem_b_req),
    .b_write_i (1'b0),
    .b_addr_i  (mem_b_addr),
    .b_wdata_i (DataWidth'(0)),
    .b_wmask_i (DataWidth'(0)),
    .b_rdata_o (mem_b_rdata),

    .cfg_i     ('0)
  );

  // Tag memory
  prim_ram_2p #(
    .Width ( 1        ),
    .Depth ( RamDepth )
  ) u_cap_ram (
    .clk_a_i   (clk_i),
    .clk_b_i   (clk_i),
    .cfg_i     ('0),
    .a_req_i   (mem_a_req),
    .a_write_i (&mem_a_we),
    .a_addr_i  (mem_a_addr[AddrWidth-1:AOff]),
    .a_wdata_i (mem_a_wcap),
    .a_wmask_i (&mem_a_we),
    .a_rdata_o (mem_a_rcap),
    .b_req_i   (mem_b_req),
    .b_write_i (1'b0),
    .b_wmask_i (1'b0),
    .b_addr_i  (mem_b_addr[AddrWidth-1:AOff]),
    .b_wdata_i (1'b0),
    .b_rdata_o (unused_mem_b_rcap)
  );

  // Single-cycle read response.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mem_a_rvalid <= '0;
      mem_b_rvalid <= '0;
    end else begin
      mem_a_rvalid <= mem_a_req & ~mem_a_we;
      mem_b_rvalid <= mem_b_req & ~mem_b_we;
    end
  end

endmodule
