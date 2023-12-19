// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Dual-port RAM with 1 cycle read/write delay, 32 bit words.
 */

`include "prim_assert.sv"

module ram_2p #(
    parameter int Depth       = 128,
    parameter int Width       = 32,
    parameter int AddrOffsetA = 2,
    parameter int AddrOffsetB = 2,
    parameter     MemInitFile = ""
) (
    input                    clk_i,
    input                    rst_ni,

    input                    a_req_i,
    input                    a_we_i,
    input        [      3:0] a_be_i,
    input        [     31:0] a_addr_i,
    input        [Width-1:0] a_wdata_i,
    output logic             a_rvalid_o,
    output logic [Width-1:0] a_rdata_o,

    input                    b_req_i,
    input                    b_we_i,
    input        [      3:0] b_be_i,
    input        [     31:0] b_addr_i,
    input        [Width-1:0] b_wdata_i,
    output logic             b_rvalid_o,
    output logic [Width-1:0] b_rdata_o
);

  localparam int Aw = $clog2(Depth);

  logic [Aw-1:0] a_addr_idx;
  assign a_addr_idx = a_addr_i[Aw - 1 + AddrOffsetA:AddrOffsetA];

  logic [31-Aw:0] unused_a_addr_parts;
  if (AddrOffsetA == 0) begin
    assign unused_a_addr_parts = a_addr_i[31:Aw];
  end else begin
    assign unused_a_addr_parts = {a_addr_i[31:Aw + AddrOffsetA],
                                  a_addr_i[AddrOffsetA - 1:0]};
  end

  logic [Aw-1:0] b_addr_idx;
  assign b_addr_idx = b_addr_i[Aw - 1 + AddrOffsetB:AddrOffsetB];

  logic [31-Aw:0] unused_b_addr_parts;
  if (AddrOffsetB == 0) begin
    assign unused_b_addr_parts = b_addr_i[31:Aw];
  end else begin
    assign unused_b_addr_parts = {b_addr_i[31:Aw + AddrOffsetB],
                                  b_addr_i[AddrOffsetB - 1:0]};
  end

  // Convert byte mask to SRAM bit mask.
  logic [Width-1:0] a_wmask;
  logic [Width-1:0] b_wmask;
  always_comb begin
    a_wmask[Width-1] = &a_be_i;
    b_wmask[Width-1] = &b_be_i;
    for (int i = 0 ; i < 4 ; i++) begin
      // mask for read data
      a_wmask[8*i+:8] = {8{a_be_i[i]}};
      b_wmask[8*i+:8] = {8{b_be_i[i]}};
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      a_rvalid_o <= '0;
      b_rvalid_o <= '0;
    end else begin
      a_rvalid_o <= a_req_i & ~a_we_i;
      b_rvalid_o <= b_req_i & ~b_we_i;
    end
  end

  prim_ram_2p #(
    .Width(Width),
    .Depth(Depth),
    .MemInitFile(MemInitFile)
  ) u_ram (
    .clk_a_i   (clk_i),
    .clk_b_i   (clk_i),
    .cfg_i     ('0),
    .a_req_i   (a_req_i),
    .a_write_i (a_we_i),
    .a_addr_i  (a_addr_idx),
    .a_wdata_i (a_wdata_i),
    .a_wmask_i (a_wmask),
    .a_rdata_o (a_rdata_o),
    .b_req_i   (b_req_i),
    .b_write_i (b_we_i),
    .b_wmask_i (b_wmask),
    .b_addr_i  (b_addr_idx),
    .b_wdata_i (b_wdata_i),
    .b_rdata_o (b_rdata_o)
  );

endmodule
