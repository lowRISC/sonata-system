// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/* File derived from hmbc_axi_top.v in the OpenHBMC project, original header is below
 * ----------------------------------------------------------------------------
 *  Project:  OpenHBMC
 *  Filename: hbmc_axi_top.v
 *  Purpose:  HyperBus memory controller AXI4 wrapper top module.
 * ----------------------------------------------------------------------------
 *  Copyright © 2020-2022, Vaagn Oganesyan <ovgn@protonmail.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 * ----------------------------------------------------------------------------
 */

`include "prim_assert.sv"

module hbmc_tl_top import tlul_pkg::*; #(
  parameter integer C_HBMC_CLOCK_HZ            = 166000000,
  parameter integer C_HBMC_FPGA_DRIVE_STRENGTH = 8,
  parameter         C_HBMC_FPGA_SLEW_RATE      = "SLOW",
  parameter integer C_HBMC_MEM_DRIVE_STRENGTH  = 46,
  parameter integer C_HBMC_CS_MAX_LOW_TIME_US  = 4,
  parameter         C_HBMC_FIXED_LATENCY       = 0,
  parameter integer C_ISERDES_CLOCKING_MODE    = 0,

  parameter         C_IDELAYCTRL_INTEGRATED    = 0,
  parameter         C_IODELAY_GROUP_ID         = "HBMC",
  parameter real    C_IODELAY_REFCLK_MHZ       = 200.0,

  parameter         C_RWDS_USE_IDELAY = 0,
  parameter         C_DQ7_USE_IDELAY  = 0,
  parameter         C_DQ6_USE_IDELAY  = 0,
  parameter         C_DQ5_USE_IDELAY  = 0,
  parameter         C_DQ4_USE_IDELAY  = 0,
  parameter         C_DQ3_USE_IDELAY  = 0,
  parameter         C_DQ2_USE_IDELAY  = 0,
  parameter         C_DQ1_USE_IDELAY  = 0,
  parameter         C_DQ0_USE_IDELAY  = 0,

  parameter [4:0]   C_RWDS_IDELAY_TAPS_VALUE = 0,
  parameter [4:0]   C_DQ7_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ6_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ5_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ4_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ3_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ2_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ1_IDELAY_TAPS_VALUE  = 0,
  parameter [4:0]   C_DQ0_IDELAY_TAPS_VALUE  = 0,

  parameter integer HyperRAMSize = 1024 * 1024 // 1 MiB
)
(
  input  clk_i,
  input  rst_ni,
  input  clk_hbmc_0,
  input  clk_hbmc_90,
  input  clk_iserdes,
  input  clk_idelay_ref,

  input  tl_h2d_t tl_i,
  output tl_d2h_t tl_o,

  /* HyperBus Interface Port */
  output wire          hb_ck_p,
  output wire          hb_ck_n,
  output wire          hb_reset_n,
  output wire          hb_cs_n,
  inout  wire          hb_rwds,
  inout  wire    [7:0] hb_dq
);

  localparam integer HyperRAMAddrW = $clog2(HyperRAMSize);

/*----------------------------------------------------------------------------------------------------------------------------*/

  logic sync_rst;

  logic idelayctrl_rdy_sync;
  logic clk_idelay;


  /* HBMC command interface */
  logic            tag_cmd_req, tag_cmd_wready;
  logic            cmd_wvalid, cmd_wready;
  logic    [31:0]  cmd_mem_addr;
  logic    [15:0]  cmd_word_cnt;
  logic            cmd_wr_not_rd;
  logic            cmd_wrap_not_incr;
  logic            cmd_ack;

  /* Upstream FIFO wires */
  logic [15:0]               ufifo_wr_data;
  logic                      ufifo_wr_last;
  logic                      ufifo_wr_ena;
  logic [top_pkg::TL_DW-1:0] ufifo_rd_dout;
  logic [9:0]                ufifo_rd_free;
  logic                      ufifo_rd_last;
  logic                      ufifo_rd_ena;
  logic                      ufifo_rd_empty;


  /* Downstream FIFO wires */
  logic [15:0]                  dfifo_rd_data;
  logic [1:0]                   dfifo_rd_strb;
  logic                         dfifo_rd_ena;
  logic [top_pkg::TL_DW-1:0]    dfifo_wr_din;
  logic [top_pkg::TL_DW/8-1:0]  dfifo_wr_strb;
  logic                         dfifo_wr_ena;
  logic                         dfifo_wr_full;

/*----------------------------------------------------------------------------------------------------------------------------*/

  /* AXI active low polarity reset inversion.
   * Positive reset polarity removes useless
   * LUT-based reset inverters, as all FPGA's
   * primitives have positive reset polarity.
   * This also improves timings. */
  always @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      sync_rst <= 1'b1;
    end else begin
      sync_rst <= 1'b0;
    end
  end

/*----------------------------------------------------------------------------------------------------------------------------*/

  generate
    if (C_IDELAYCTRL_INTEGRATED) begin

      wire    idelayctrl_rdy;
      wire    idelayctrl_rst;


      hbmc_arst_sync #
      (
          /* Current module requires min
           * 60ns of reset pulse width,
           * 32 stage synchronizer will
           * be enough for AXI clock
           * frequencies < 500MHz */
          .C_SYNC_STAGES ( 32 )
      )
      hbmc_arst_sync_idelayctrl
      (
          .clk   ( clk_i     ),
          .arst  ( sync_rst   ),
          .rst   ( idelayctrl_rst )
      );


      (* IODELAY_GROUP = C_IODELAY_GROUP_ID *)
      IDELAYCTRL
      IDELAYCTRL_inst
      (
          .RST    ( idelayctrl_rst ),
          .REFCLK ( clk_idelay_ref ),
          .RDY    ( idelayctrl_rdy )
      );


      hbmc_bit_sync #
      (
          .C_SYNC_STAGES  ( 3     ),
          .C_RESET_STATE  ( 1'b0  )
      )
      hbmc_bit_sync_idelayctrl_rdy
      (
          .arst   ( sync_rst        ),
          .clk    ( clk_i          ),
          .d      ( idelayctrl_rdy      ),
          .q      ( idelayctrl_rdy_sync )
      );

      assign clk_idelay = clk_idelay_ref;

    end else begin
      assign idelayctrl_rdy_sync = 1'b1;
      assign clk_idelay = 1'b0;
    end
  endgenerate

/*----------------------------------------------------------------------------------------------------------------------------*/

  wire    hbmc_rst_sync;


  hbmc_arst_sync #
  (
      .C_SYNC_STAGES ( 3 )
  )
  hbmc_arst_sync_inst
  (
      .clk  ( clk_hbmc_0    ),
      .arst ( sync_rst      ),
      .rst  ( hbmc_rst_sync )
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  // Metadata from inbound tilelink transactions that needs to be saved to produce the response
  typedef struct packed {
    logic [top_pkg::TL_AIW-1:0] tl_source;
    logic [top_pkg::TL_SZW-1:0] tl_size;
    logic                       cmd_wr_not_rd;
  } tl_req_info_t;

  tl_req_info_t tl_req_fifo_wdata, tl_req_fifo_rdata;

  logic tl_req_fifo_wvalid, tl_req_fifo_wready;
  logic tl_req_fifo_rvalid, tl_req_fifo_rready;

  logic tl_tag_bit, tl_a_ready;

  tl_d2h_t tl_o_int;

  // Logic for handling incoming tilelink requests
  always_comb begin
    cmd_wvalid         = 1'b0;
    cmd_wr_not_rd      = 1'b0;
    tag_cmd_req        = 1'b0;
    dfifo_wr_ena       = 1'b0;
    tl_req_fifo_wvalid = 1'b0;
    tl_a_ready         = 1'b0;

    if (tl_i.a_valid && tl_req_fifo_wready && tag_cmd_wready && cmd_wready &&
      (tl_i.a_opcode == Get || ~dfifo_wr_full)) begin
      // We can accept an incoming tilelink transaction when we've got space in the hyperram, tag
      // and tilelink request FIFOs. If we're taking in a write transaction we also need space in the
      // downstream FIFO(dfifo) for the write data

      // Write to the relevant FIFOs and indicate ready on tilelink A channel
      cmd_wvalid         = 1'b1;
      tag_cmd_req        = 1'b1;
      tl_req_fifo_wvalid = 1'b1;
      tl_a_ready         = 1'b1;

      if (tl_i.a_opcode != Get) begin
        cmd_wr_not_rd    = 1'b1;
        dfifo_wr_ena     = 1'b1;
      end
    end
  end

  assign dfifo_wr_strb = tl_i.a_mask;
  assign dfifo_wr_din  = tl_i.a_data;

  assign tl_req_fifo_wdata = '{
    tl_source     : tl_i.a_source,
    tl_size       : tl_i.a_size,
    cmd_wr_not_rd : cmd_wr_not_rd
  };

  // Logic for sending out tilelink responses
  always_comb begin
    tl_o_int           = '0;
    tl_req_fifo_rready = 1'b0;
    ufifo_rd_ena       = 1'b0;

    if (tl_req_fifo_rvalid) begin
      // We have an incoming request that needs a response
      if (tl_req_fifo_rdata.cmd_wr_not_rd) begin
        // If it's a write then return an immediate response (early response is reasonable as any
        // read that could observe the memory cannot occur until the write has actually happened)
        tl_o_int.d_valid   = 1'b1;
        tl_req_fifo_rready = tl_i.d_ready;
      end else begin
        // Otherwise wait until we have read data to return
        tl_o_int.d_valid   = ~ufifo_rd_empty;
        // Only dequeue read data from the upstream FIFO (ufifo) and request FIFO when the tilelink
        // D channel is ready
        ufifo_rd_ena       = tl_i.d_ready & ~ufifo_rd_empty;
        tl_req_fifo_rready = ufifo_rd_ena;
      end
    end

    tl_o_int.d_opcode          = tl_req_fifo_rdata.cmd_wr_not_rd ? AccessAck : AccessAckData;
    tl_o_int.d_size            = tl_req_fifo_rdata.tl_size;
    tl_o_int.d_source          = tl_req_fifo_rdata.tl_source;
    tl_o_int.d_data            = ufifo_rd_dout;
    tl_o_int.d_user.capability = tl_tag_bit;
    tl_o_int.a_ready           = tl_a_ready;
  end

  // Generate integrity for outgoing response
  tlul_rsp_intg_gen u_tlul_rsp_intg_gen (
    .tl_i(tl_o_int),
    .tl_o(tl_o)
  );

  localparam TL_REQ_FIFO_DEPTH = 4;

  prim_fifo_sync #(
    .Width($bits(tl_req_info_t)),
    .Depth(TL_REQ_FIFO_DEPTH),
    .Pass(1'b0)
  ) u_tl_req_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (1'b0),
    .wvalid_i (tl_req_fifo_wvalid),
    .wready_o (tl_req_fifo_wready),
    .wdata_i  (tl_req_fifo_wdata),
    .rvalid_o (tl_req_fifo_rvalid),
    .rready_i (tl_req_fifo_rready),
    .rdata_o  (tl_req_fifo_rdata),

    .full_o  (),
    .depth_o (),
    .err_o   ()
  );

  assign cmd_mem_addr      = {{(33 - HyperRAMAddrW){1'b0}}, tl_i.a_address[HyperRAMAddrW-1:1]};
  assign cmd_word_cnt      = 16'd2;
  assign cmd_wrap_not_incr = 1'b0;

/*----------------------------------------------------------------------------------------------------------------------------*/

  localparam  BUS_SYNC_WIDTH = 32 + 16 + 1 + 1;

  logic            cmd_rvalid, cmd_rready;
  logic    [31:0]  cmd_mem_addr_dst;
  logic    [15:0]  cmd_word_cnt_dst;
  logic            cmd_wr_not_rd_dst;
  logic            cmd_wrap_not_incr_dst;


  logic    [BUS_SYNC_WIDTH - 1:0]  cmd_wdata, cmd_rdata;

  assign cmd_wdata = {cmd_mem_addr, cmd_word_cnt, cmd_wr_not_rd, cmd_wrap_not_incr};
  assign {cmd_mem_addr_dst, cmd_word_cnt_dst, cmd_wr_not_rd_dst, cmd_wrap_not_incr_dst} = cmd_rdata;

  prim_fifo_async #(
    .Width(BUS_SYNC_WIDTH),
    .Depth(2)
  ) u_hbmc_cmd_fifo (
    .clk_wr_i (clk_i),
    .rst_wr_ni(~sync_rst),
    .wvalid_i (cmd_wvalid),
    .wready_o (cmd_wready),
    .wdata_i  (cmd_wdata),
    .wdepth_o (),

    .clk_rd_i (clk_hbmc_0),
    .rst_rd_ni(~hbmc_rst_sync),
    .rvalid_o (cmd_rvalid),
    .rready_i (cmd_rready),
    .rdata_o  (cmd_rdata),
    .rdepth_o ()
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  hbmc_ctrl #
  (
      .C_AXI_DATA_WIDTH           ( top_pkg::TL_DW             ),
      .C_HBMC_CLOCK_HZ            ( C_HBMC_CLOCK_HZ            ),
      .C_HBMC_FPGA_DRIVE_STRENGTH ( C_HBMC_FPGA_DRIVE_STRENGTH ),
      .C_HBMC_FPGA_SLEW_RATE      ( C_HBMC_FPGA_SLEW_RATE      ),
      .C_HBMC_MEM_DRIVE_STRENGTH  ( C_HBMC_MEM_DRIVE_STRENGTH  ),
      .C_HBMC_CS_MAX_LOW_TIME_US  ( C_HBMC_CS_MAX_LOW_TIME_US  ),
      .C_HBMC_FIXED_LATENCY       ( C_HBMC_FIXED_LATENCY       ),
      .C_ISERDES_CLOCKING_MODE    ( C_ISERDES_CLOCKING_MODE    ),
      .C_IODELAY_GROUP_ID         ( C_IODELAY_GROUP_ID         ),
      .C_IODELAY_REFCLK_MHZ       ( C_IODELAY_REFCLK_MHZ       ),

      .C_RWDS_USE_IDELAY          ( C_RWDS_USE_IDELAY          ),
      .C_DQ7_USE_IDELAY           ( C_DQ7_USE_IDELAY           ),
      .C_DQ6_USE_IDELAY           ( C_DQ6_USE_IDELAY           ),
      .C_DQ5_USE_IDELAY           ( C_DQ5_USE_IDELAY           ),
      .C_DQ4_USE_IDELAY           ( C_DQ4_USE_IDELAY           ),
      .C_DQ3_USE_IDELAY           ( C_DQ3_USE_IDELAY           ),
      .C_DQ2_USE_IDELAY           ( C_DQ2_USE_IDELAY           ),
      .C_DQ1_USE_IDELAY           ( C_DQ1_USE_IDELAY           ),
      .C_DQ0_USE_IDELAY           ( C_DQ0_USE_IDELAY           ),

      .C_RWDS_IDELAY_TAPS_VALUE   ( C_RWDS_IDELAY_TAPS_VALUE   ),
      .C_DQ7_IDELAY_TAPS_VALUE    ( C_DQ7_IDELAY_TAPS_VALUE    ),
      .C_DQ6_IDELAY_TAPS_VALUE    ( C_DQ6_IDELAY_TAPS_VALUE    ),
      .C_DQ5_IDELAY_TAPS_VALUE    ( C_DQ5_IDELAY_TAPS_VALUE    ),
      .C_DQ4_IDELAY_TAPS_VALUE    ( C_DQ4_IDELAY_TAPS_VALUE    ),
      .C_DQ3_IDELAY_TAPS_VALUE    ( C_DQ3_IDELAY_TAPS_VALUE    ),
      .C_DQ2_IDELAY_TAPS_VALUE    ( C_DQ2_IDELAY_TAPS_VALUE    ),
      .C_DQ1_IDELAY_TAPS_VALUE    ( C_DQ1_IDELAY_TAPS_VALUE    ),
      .C_DQ0_IDELAY_TAPS_VALUE    ( C_DQ0_IDELAY_TAPS_VALUE    )
  )
  hbmc_ctrl_inst
  (
      .rst                ( hbmc_rst_sync         ),
      .clk_hbmc_0         ( clk_hbmc_0            ),
      .clk_hbmc_90        ( clk_hbmc_90           ),
      .clk_iserdes        ( clk_iserdes           ),
      .clk_idelay_ref     ( clk_idelay            ),

      .cmd_valid          ( cmd_rvalid            ),
      .cmd_ready          ( cmd_rready            ),
      .cmd_mem_addr       ( cmd_mem_addr_dst      ),
      .cmd_word_count     ( cmd_word_cnt_dst      ),
      .cmd_wr_not_rd      ( cmd_wr_not_rd_dst     ),
      .cmd_wrap_not_incr  ( cmd_wrap_not_incr_dst ),

      .ufifo_data         ( ufifo_wr_data         ),
      .ufifo_last         ( ufifo_wr_last         ),
      .ufifo_we           ( ufifo_wr_ena          ),

      .dfifo_data         ( dfifo_rd_data         ),
      .dfifo_strb         ( dfifo_rd_strb         ),
      .dfifo_re           ( dfifo_rd_ena          ),

      .hb_ck_p            ( hb_ck_p               ),
      .hb_ck_n            ( hb_ck_n               ),
      .hb_reset_n         ( hb_reset_n            ),
      .hb_cs_n            ( hb_cs_n               ),
      .hb_rwds            ( hb_rwds               ),
      .hb_dq              ( hb_dq                 )
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  /* Upstream data FIFO */
  hbmc_ufifo #
  (
      .DATA_WIDTH ( top_pkg::TL_DW )
  )
  hbmc_ufifo_inst
  (
      .fifo_arst      ( sync_rst   ),

      .fifo_wr_clk    ( clk_hbmc_0     ),
      .fifo_wr_din    ( ufifo_wr_data  ),
      .fifo_wr_last   ( ufifo_wr_last  ),
      .fifo_wr_ena    ( ufifo_wr_ena   ),
      .fifo_wr_full   ( /*----NC----*/ ),

      .fifo_rd_clk    ( clk_i     ),
      .fifo_rd_dout   ( ufifo_rd_dout  ),
      .fifo_rd_free   ( /*----NC----*/ ),
      .fifo_rd_last   ( ufifo_rd_last  ),
      .fifo_rd_ena    ( ufifo_rd_ena   ),
      .fifo_rd_empty  ( ufifo_rd_empty )
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  /* Downstream data FIFO */
  hbmc_dfifo #
  (
      .DATA_WIDTH ( top_pkg::TL_DW )
  )
  hbmc_dfifo_inst
  (
      .fifo_arst      ( sync_rst   ),

      .fifo_wr_clk    ( clk_i     ),
      .fifo_wr_din    ( dfifo_wr_din   ),
      .fifo_wr_strb   ( dfifo_wr_strb  ),
      .fifo_wr_ena    ( dfifo_wr_ena   ),
      .fifo_wr_full   ( dfifo_wr_full  ),

      .fifo_rd_clk    ( clk_hbmc_0     ),
      .fifo_rd_dout   ( dfifo_rd_data  ),
      .fifo_rd_strb   ( dfifo_rd_strb  ),
      .fifo_rd_ena    ( dfifo_rd_ena   ),
      .fifo_rd_empty  ( /*----NC----*/ )
  );


/*----------------------------------------------------------------------------------------------------------------------------*/
  // Capability tag handling
  // Command FIFO provides reads and writes from tilelink requests. Writes just happen without any
  // response and reads writes to rdata_fifo to be picked up by the tilelink response.

  // 1 tag bit per 64 bits so divide HyperRAMSize by 8
  localparam TAG_ADDR_W = $clog2(HyperRAMSize) - 3;
  localparam TAG_FIFO_DEPTH = 4;

  typedef struct packed {
    logic [TAG_ADDR_W-1:0] addr;
    logic                  write;
    logic                  wdata;
  } tag_cmd_t;

  tag_cmd_t tag_cmd_in, tag_cmd_out;
  logic tag_cmd_valid;

  logic tag_rdata;
  logic tag_rdata_valid_q, tag_rdata_valid_d;
  logic tag_rdata_fifo_rvalid;

  assign tag_cmd_in.addr  = cmd_mem_addr[TAG_ADDR_W + 1:2];
  assign tag_cmd_in.write = cmd_wr_not_rd;
  assign tag_cmd_in.wdata = tl_i.a_user.capability;

  prim_fifo_sync #(
    .Width($bits(tag_cmd_t)),
    .Depth(TAG_FIFO_DEPTH),
    .Pass(1'b0)
  ) u_tag_cmd_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (1'b0),
    .wvalid_i (tag_cmd_req),
    .wready_o (tag_cmd_wready),
    .wdata_i  (tag_cmd_in),
    .rvalid_o (tag_cmd_valid),
    .rready_i (1'b1),
    .rdata_o  (tag_cmd_out),

    .full_o  (),
    .depth_o (),
    .err_o   ()
  );

  prim_fifo_sync #(
    .Width(1),
    .Depth(TAG_FIFO_DEPTH),
    .Pass(1'b0)
  ) u_tag_rdata_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (1'b0),
    .wvalid_i (tag_rdata_valid_q),
    .wready_o (),
    .wdata_i  (tag_rdata),
    .rvalid_o (tag_rdata_fifo_rvalid),
    .rready_i (ufifo_rd_ena),
    .rdata_o  (tl_tag_bit),

    .full_o  (),
    .depth_o (),
    .err_o   ()
  );

  `ASSERT(always_tag_rdata_valid_when_read_data_response, ufifo_rd_ena |-> tag_rdata_fifo_rvalid)

  assign tag_rdata_valid_d = tag_cmd_valid & ~tag_cmd_out.write;

  always @(posedge clk_i, negedge rst_ni) begin
    if (~rst_ni) begin
      tag_rdata_valid_q <= 1'b0;
    end else begin
      tag_rdata_valid_q <= tag_rdata_valid_d;
    end
  end

  prim_ram_1p #(
    .Width(1),
    .Depth(2 ** TAG_ADDR_W)
  ) u_tag_ram (
    .clk_i   (clk_i),
    .req_i   (tag_cmd_valid),
    .write_i (tag_cmd_out.write),
    .addr_i  (tag_cmd_out.addr),
    .wdata_i (tag_cmd_out.wdata),
    .wmask_i ('1),
    .rdata_o (tag_rdata),
    .cfg_i   ('0)
  );
endmodule
