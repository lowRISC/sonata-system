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

  parameter int unsigned NumPorts = 2,
  // Mapped size of the HyperRAM, in bytes.
  parameter int unsigned HyperRAMSize = 8 * 1024 * 1024, // 8 MiB
  // Mapped portion of the HyperRAM that can store capabilities, in bytes.
  parameter int unsigned HyperRAMTagSize = 4 * 1024 * 1024 // 4 MiB
)
(
  input  clk_i,
  input  rst_ni,
  input  clk_hbmc_0,
  input  clk_hbmc_90,
  input  rst_hbmc_ni,
  input  clk_iserdes,
  input  clk_idelay_ref,

  input  tl_h2d_t tl_i[NumPorts],
  output tl_d2h_t tl_o[NumPorts],

  /* HyperBus Interface Port */
  output wire          hb_ck_p,
  output wire          hb_ck_n,
  output wire          hb_reset_n,
  output wire          hb_cs_n,
  inout  wire          hb_rwds,
  inout  wire    [7:0] hb_dq
);

  // Maximum number of data buffers per port.
  localparam int unsigned MaxBufs = 4;

  // Two TL-UL access ports.
  localparam int unsigned PortD = 0;
  localparam int unsigned PortI = 1;

  // Width of port ID numbers, in bits.
  localparam int unsigned PortIDWidth = $clog2(NumPorts);
  localparam int unsigned Log2MaxBufs = $clog2(MaxBufs);
  // Up to 4 outstanding requests from a single buffer +1 for invalidation, plus
  // bits to identify the buffer number, and a further bit for the port number of
  // the requester.
  localparam int unsigned SeqWidth = PortIDWidth + Log2MaxBufs + 3;
  // Width of HyperRAM address, in bits.
  localparam int unsigned HyperRAMAddrW = $clog2(HyperRAMSize);
  // Address width of the portion that can store capabilities, in bits.
  localparam int unsigned HyperRAMTagAddrW = $clog2(HyperRAMTagSize);
  // LSB of word address.
  localparam int unsigned ABIT = $clog2(top_pkg::TL_DW / 8);
  // Use 32-byte bursts for performance, whilst reducing the penalty of wasted burst reads.
  localparam int unsigned Log2BurstLen = 5;

/*----------------------------------------------------------------------------------------------------------------------------*/

  /* We need the Upstream FIFO from the HyperRAM controller core to accommodate an entire
     burst read. Upstream transfers write 16 bits into the upstream FIFO every cycle, but the
     Sonata system clock is only 40% of that clock frequency. Additionally, because of the CDC
     into the slower clock domain, it can take 4 system clock cycles to collect the first word.
  */
  localparam int unsigned UDataWidth = top_pkg::TL_DW;
  localparam int unsigned UFIFODepth = 2 << (Log2BurstLen - ABIT);

  /* The Downstream FIFO to the HyperRAM controller must be wide enough and deep enough to
   * accommodate all of the write data for a burst. The write coealescing logic in `hyperram_wrbuf`
   * relies upon being to push data words before issuing the write command, and once the write
   * command is accepted, data will be popped faster than the system clock can supply it.
   */
  localparam int unsigned DDataWidth = top_pkg::TL_DW;
  localparam int unsigned DFIFODepth = 1 << (Log2BurstLen - ABIT);

  logic idelayctrl_rdy_sync;
  logic clk_idelay;

  /* Tag memory interface */
  logic                      tag_cmd_req;
  logic                      tag_rdata_rready;

  /* HBMC command interface */
  logic [NumPorts-1:0]                       cmd_req;
  logic [NumPorts-1:0]                       cmd_wready;
  logic [NumPorts-1:0][HyperRAMAddrW-1:ABIT] cmd_mem_addr;
  logic [NumPorts-1:0][Log2BurstLen-ABIT:0]  cmd_word_cnt;  // Bus words.
  logic [NumPorts-1:0]                       cmd_wr_not_rd;
  logic [NumPorts-1:0]                       cmd_wrap_not_incr;
  logic [NumPorts-1:0][SeqWidth-1:0]         cmd_seq;

  /* Upstream FIFO wires */
  logic [15:0]               ufifo_wr_data;
  logic                      ufifo_wr_last;
  logic                      ufifo_wr_ena;
  logic [UDataWidth-1:0]     ufifo_rd_dout;
  logic [9:0]                ufifo_rd_free;
  logic                      ufifo_rd_last;
  logic                      ufifo_rd_ena;
  logic                      ufifo_rd_empty;


  /* Downstream FIFO wires */
  logic [15:0]               dfifo_rd_data;
  logic [1:0]                dfifo_rd_strb;
  logic                      dfifo_rd_ena;
  logic [DDataWidth-1:0]     dfifo_wr_din;
  logic [DDataWidth/8-1:0]   dfifo_wr_strb;
  logic                      dfifo_wr_ena;
  logic                      dfifo_wr_full;


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
          .clk   ( clk_i          ),
          .arst  ( ~rst_ni        ),
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
          .arst   ( ~rst_ni             ),
          .clk    ( clk_i               ),
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
  // FIFOs on the TL-UL ports break a combinatorial loop between the instruction fetch and LSU ports
  // of the CPU since the HyperRAM is presented to both ports, but without adding latency to the
  // request or response.
  tlul_pkg::tl_h2d_t tl_i_int[NumPorts];
  tlul_pkg::tl_d2h_t tl_o_int[NumPorts];

  for (genvar p = 0; p < NumPorts; p++) begin : gen_tlul_fifos
    tlul_fifo_sync #(
      .ReqPass      (1'b1),  // Do not add latency.
      .RspPass      (1'b1),
      .ReqDepth     (1),  // No need for more than a single slot.
      .RspDepth     (1)
    ) u_tl_fifo(
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .tl_h_i       (tl_i[p]),
      .tl_h_o       (tl_o[p]),
      .tl_d_o       (tl_i_int[p]),
      .tl_d_i       (tl_o_int[p]),
      .spare_req_i  ('0),
      .spare_req_o  (),
      .spare_rsp_i  ('0),
      .spare_rsp_o  ()
    );
  end : gen_tlul_fifos

/*----------------------------------------------------------------------------------------------------------------------------*/
  // TL-UL Access ports.

  logic [SeqWidth-1:0] ufifo_rd_seq;

  logic [NumPorts-1:0] ufifo_all_rd_ena;

  logic [NumPorts-1:0][DDataWidth-1:0]    dfifo_all_wr_din;
  logic [NumPorts-1:0][DDataWidth/8-1:0]  dfifo_all_wr_strb;
  logic [NumPorts-1:0]                    dfifo_all_wr_ena;
  logic [NumPorts-1:0]                    dfifo_all_wr_full;

  logic [NumPorts-1:0] tag_all_cmd_req;
  logic [NumPorts-1:0][HyperRAMTagAddrW-1:ABIT] tag_all_cmd_mem_addr;
  logic [NumPorts-1:0] tag_all_cmd_wr_not_rd;
  logic [NumPorts-1:0] tag_all_rdata_rready;

  logic tl_tag_bit;
  logic [NumPorts-1:0] tag_all_cmd_wcap;

  // Write notifications.
  logic [NumPorts-1:0] wr_notify_out;
  logic [NumPorts-1:0] wr_notify_in;
  logic [NumPorts-1:0][HyperRAMAddrW-1:ABIT] wr_notify_addr_out;
  logic [NumPorts-1:0][HyperRAMAddrW-1:ABIT] wr_notify_addr_in;
  logic [NumPorts-1:0][top_pkg::TL_DBW-1:0] wr_notify_mask_out;
  logic [NumPorts-1:0][top_pkg::TL_DBW-1:0] wr_notify_mask_in;
  logic [NumPorts-1:0][top_pkg::TL_DW-1:0] wr_notify_data_out;
  logic [NumPorts-1:0][top_pkg::TL_DW-1:0] wr_notify_data_in;

  if (NumPorts > 1) begin : gen_wr_notify
    // Instruction port requires notifications of writes occurring on the Data port.
    assign wr_notify_in[PortI]      = wr_notify_out[PortD];
    assign wr_notify_addr_in[PortI] = wr_notify_addr_out[PortD];
    assign wr_notify_mask_in[PortI] = wr_notify_mask_out[PortD];
    assign wr_notify_data_in[PortI] = wr_notify_data_out[PortD];
    // Writes shall not occur on the Instruction port.
    assign {wr_notify_in[PortD], wr_notify_addr_in[PortD]} = '0;
  end else begin : gen_no_wr_notify
    assign wr_notify_in = '0;
    assign wr_notify_addr_in = '0;
    assign wr_notify_mask_in = '0;
    assign wr_notify_data_in = '0;
  end

  for (genvar p = 0; p < NumPorts; p++) begin : gen_ports
    hbmc_tl_port #(
      .HyperRAMAddrW    (HyperRAMAddrW),
      .HyperRAMTagAddrW (HyperRAMTagAddrW),
      .Log2BurstLen     (Log2BurstLen),
      .NumBufs          (MaxBufs),
      .PortIDWidth      (PortIDWidth),
      .Log2MaxBufs      (Log2MaxBufs),
      .SeqWidth         (SeqWidth),
      .SupportWrites    (p == PortD)  // Only the data port supports writing.
    ) u_port(
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),

      // Port numbers.
      .portid_i           (PortIDWidth'(p)),

      // TL-UL interface.
      .tl_i               (tl_i_int[p]),
      .tl_o               (tl_o_int[p]),

      // Write notification input.
      .wr_notify_i        (wr_notify_in[p]),
      .wr_notify_addr_i   (wr_notify_addr_in[p]),
      .wr_notify_mask_i   (wr_notify_mask_in[p]),
      .wr_notify_data_i   (wr_notify_data_in[p]),

      // Write notification output.
      .wr_notify_o        (wr_notify_out[p]),
      .wr_notify_addr_o   (wr_notify_addr_out[p]),
      .wr_notify_mask_o   (wr_notify_mask_out[p]),
      .wr_notify_data_o   (wr_notify_data_out[p]),

      // Command data to the HyperRAM controller.
      .cmd_req_o          (cmd_req[p]),
      .cmd_wready_i       (cmd_wready[p]),
      .cmd_mem_addr_o     (cmd_mem_addr[p]),
      .cmd_word_cnt_o     (cmd_word_cnt[p]),
      .cmd_wr_not_rd_o    (cmd_wr_not_rd[p]),
      .cmd_wrap_not_incr_o(cmd_wrap_not_incr[p]),
      .cmd_seq_o          (cmd_seq[p]),
      .tag_cmd_req        (tag_all_cmd_req[p]),
      .tag_cmd_mem_addr   (tag_all_cmd_mem_addr[p]),
      .tag_cmd_wr_not_rd  (tag_all_cmd_wr_not_rd[p]),
      .tag_cmd_wcap       (tag_all_cmd_wcap[p]),
      .dfifo_wr_ena_o     (dfifo_all_wr_ena[p]),
      .dfifo_wr_full_i    (dfifo_all_wr_full[p]),
      .dfifo_wr_strb_o    (dfifo_all_wr_strb[p]),
      .dfifo_wr_din_o     (dfifo_all_wr_din[p]),

      // Read data from the HyperRAM controller.
      .ufifo_rd_ena       (ufifo_all_rd_ena[p]),
      .ufifo_rd_empty     (ufifo_rd_empty),
      .ufifo_rd_dout      (ufifo_rd_dout),
      .ufifo_rd_seq       (ufifo_rd_seq),
      .ufifo_rd_last      (ufifo_rd_last),

      // Tag read data interface.
      .tag_rdata_rready   (tag_all_rdata_rready[p]),
      .tl_tag_bit         (tl_tag_bit)
    );
  end : gen_ports

  // Upstream FIFO traffic is presented to all ports, but only the one indicated by the sequence
  // number within the FIFO entry shall consume it.
  assign ufifo_rd_ena = |ufifo_all_rd_ena;

  // Downstream FIFO traffic comes from the Data port, since this is the only port that performs
  // writes.
  assign dfifo_wr_ena  = dfifo_all_wr_ena[PortD];
  assign dfifo_wr_strb = dfifo_all_wr_strb[PortD];
  assign dfifo_wr_din  = dfifo_all_wr_din[PortD];
  assign dfifo_all_wr_full = {NumPorts{dfifo_wr_full}};

  // Only the Data port requires tag bits.
  assign tag_cmd_req      = tag_all_cmd_req[PortD];
  assign tag_rdata_rready = tag_all_rdata_rready[PortD];

/*----------------------------------------------------------------------------------------------------------------------------*/
  // Arbitrate amongst the access ports.
  localparam  BUS_SYNC_WIDTH = (HyperRAMAddrW - ABIT)     // Address, in terms of TL-UL bus words.
                             + (Log2BurstLen + 1 - ABIT)  // Number of TL-UL bus words.
                             + 1 + 1 + SeqWidth;          // Write/Read, Wrap/linear, Sequence no.
  logic cmd_fifo_wvalid;
  logic cmd_fifo_wready;
  logic [BUS_SYNC_WIDTH-1:0] cmd_fifo_wdata;

  if (NumPorts > 1) begin : gen_multi_port
    logic [BUS_SYNC_WIDTH-1:0] cmd_wdata[NumPorts];
    always_comb begin
      for (int unsigned p = 0; p < NumPorts; p++) begin
        cmd_wdata[p] = {cmd_mem_addr[p], cmd_word_cnt[p], cmd_wr_not_rd[p], cmd_wrap_not_incr[p], cmd_seq[p]};
      end
    end

    prim_arbiter_fixed #(
      .N    (NumPorts),
      .DW   (BUS_SYNC_WIDTH)
    ) u_cmd_arbiter(
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),

      .req_i   (cmd_req),
      .data_i  (cmd_wdata),
      .gnt_o   (cmd_wready),
      .idx_o   (),  // Not used.

      .valid_o (cmd_fifo_wvalid),
      .data_o  (cmd_fifo_wdata),
      .ready_i (cmd_fifo_wready)
    );
  end else begin : gen_single_port
    assign cmd_fifo_wvalid = cmd_req & cmd_fifo_wready;
    assign cmd_wready = cmd_fifo_wready;
    assign cmd_fifo_wdata = {cmd_mem_addr, cmd_word_cnt, cmd_wr_not_rd, cmd_wrap_not_incr, cmd_seq};
  end

/*----------------------------------------------------------------------------------------------------------------------------*/

  logic                        cmd_rvalid, cmd_rready;
  logic [HyperRAMAddrW-1:ABIT] cmd_mem_addr_dst;
  logic    [Log2BurstLen:ABIT] cmd_word_cnt_dst;
  logic                        cmd_wr_not_rd_dst;
  logic                        cmd_wrap_not_incr_dst;
  logic         [SeqWidth-1:0] cmd_seq_dst;

  logic   [BUS_SYNC_WIDTH-1:0] cmd_rdata;

  assign {cmd_mem_addr_dst, cmd_word_cnt_dst, cmd_wr_not_rd_dst, cmd_wrap_not_incr_dst, cmd_seq_dst} = cmd_rdata;

  prim_fifo_async #(
    .Width(BUS_SYNC_WIDTH),
    .Depth(4)
  ) u_hbmc_cmd_fifo (
    .clk_wr_i (clk_i),
    .rst_wr_ni(rst_ni),
    .wvalid_i (cmd_fifo_wvalid),
    .wready_o (cmd_fifo_wready),
    .wdata_i  (cmd_fifo_wdata),
    .wdepth_o (),

    .clk_rd_i (clk_hbmc_0),
    .rst_rd_ni(rst_hbmc_ni),
    .rvalid_o (cmd_rvalid),
    .rready_i (cmd_rready),
    .rdata_o  (cmd_rdata),
    .rdepth_o ()
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  // Widened address and word count, converting from TL-UL bus words to 16-bit HyperRAM words.
  logic [31:0] cmd_mem_addr_full;
  logic [15:0] cmd_word_cnt_full;
  assign cmd_mem_addr_full = 32'({cmd_mem_addr_dst, {(ABIT-1){1'b0}}});
  assign cmd_word_cnt_full = 16'({cmd_word_cnt_dst, {(ABIT-1){1'b0}}});

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
      .rst                ( ~rst_hbmc_ni          ),
      .clk_hbmc_0         ( clk_hbmc_0            ),
      .clk_hbmc_90        ( clk_hbmc_90           ),
      .clk_iserdes        ( clk_iserdes           ),
      .clk_idelay_ref     ( clk_idelay            ),

      .cmd_valid          ( cmd_rvalid            ),
      .cmd_ready          ( cmd_rready            ),
      .cmd_mem_addr       ( cmd_mem_addr_full     ),
      .cmd_word_count     ( cmd_word_cnt_full     ),
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

  /* Return the sequence number from the command so that the read data may be steered appropriately */
  logic [SeqWidth-1:0] ufifo_wr_seq;
  always_ff @(posedge clk_hbmc_0) begin
    if (cmd_rvalid & cmd_rready) ufifo_wr_seq <= cmd_seq_dst;
  end

  /* Upstream data FIFO */
  hbmc_ufifo #
  (
      .DataWidth ( UDataWidth ),
      .FIFODepth ( UFIFODepth ),
      .SeqWidth  ( SeqWidth   )
  )
  hbmc_ufifo_inst
  (
      .fifo_wr_clk    ( clk_hbmc_0      ),
      .fifo_wr_nrst   ( rst_hbmc_ni     ),
      .fifo_wr_din    ( ufifo_wr_data   ),
      .fifo_wr_seq    ( ufifo_wr_seq    ),
      .fifo_wr_last   ( ufifo_wr_last   ),
      .fifo_wr_ena    ( ufifo_wr_ena    ),
      .fifo_wr_full   ( /*----NC----*/  ),

      .fifo_rd_clk    ( clk_i           ),
      .fifo_rd_nrst   ( rst_ni          ),
      .fifo_rd_dout   ( ufifo_rd_dout   ),
      .fifo_rd_seq    ( ufifo_rd_seq    ),
      .fifo_rd_last   ( ufifo_rd_last   ),
      .fifo_rd_ena    ( ufifo_rd_ena    ),
      .fifo_rd_empty  ( ufifo_rd_empty  )
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  /* Downstream data FIFO */
  hbmc_dfifo #
  (
      .DataWidth ( DDataWidth ),
      .FIFODepth ( DFIFODepth )
  )
  hbmc_dfifo_inst
  (

      .fifo_wr_clk    ( clk_i          ),
      .fifo_wr_nrst   ( rst_ni         ),
      .fifo_wr_din    ( dfifo_wr_din   ),
      .fifo_wr_strb   ( dfifo_wr_strb  ),
      .fifo_wr_ena    ( dfifo_wr_ena   ),
      .fifo_wr_full   ( dfifo_wr_full  ),

      .fifo_rd_clk    ( clk_hbmc_0     ),
      .fifo_rd_nrst   ( rst_hbmc_ni    ),
      .fifo_rd_dout   ( dfifo_rd_data  ),
      .fifo_rd_strb   ( dfifo_rd_strb  ),
      .fifo_rd_ena    ( dfifo_rd_ena   ),
      .fifo_rd_empty  ( /*----NC----*/ )
  );


/*----------------------------------------------------------------------------------------------------------------------------*/
  // Capability tag handling
  // Command FIFO provides reads and writes from tilelink requests. Writes just happen without any
  // response and reads writes to rdata_fifo to be picked up by the tilelink response.

  // 1 tag bit per 64 bits so divide HyperRAMTagSize by 8
  localparam TAG_ADDR_W = HyperRAMTagAddrW - 3;
  localparam TAG_FIFO_DEPTH = 4;

  typedef struct packed {
    logic [TAG_ADDR_W-1:0] addr;
    logic                  write;
    logic                  wdata;
  } tag_cmd_t;

  logic tag_rdata;
  logic tag_rdata_valid_q, tag_rdata_valid_d;
  logic tag_rdata_fifo_rvalid;

  // Only the Data port requires capability tags.
  tag_cmd_t tag_cmd;
  assign tag_cmd.addr  = tag_all_cmd_mem_addr[PortD][HyperRAMTagAddrW-1:ABIT+1];
  assign tag_cmd.write = tag_all_cmd_wr_not_rd[PortD];
  assign tag_cmd.wdata = tag_all_cmd_wcap[PortD];

  prim_fifo_sync #(
    .Width(1),
    .Depth(TAG_FIFO_DEPTH),
    // Pass through is required when a Read operation hits in the internal buffer, since that
    // takes only a single cycle.
    .Pass(1'b1)
  ) u_tag_rdata_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (1'b0),
    .wvalid_i (tag_rdata_valid_q),
    .wready_o (),
    .wdata_i  (tag_rdata),
    .rvalid_o (tag_rdata_fifo_rvalid),
    .rready_i (tag_rdata_rready),
    .rdata_o  (tl_tag_bit),

    .full_o  (),
    .depth_o (),
    .err_o   ()
  );

  // TODO: Probably wants some refinement/augmentation.
  // `ASSERT(always_tag_rdata_valid_when_read_data_response, ufifo_rd_ena |-> tag_rdata_fifo_rvalid)

  assign tag_rdata_valid_d = tag_cmd_req & ~tag_cmd.write;

  always_ff @(posedge clk_i, negedge rst_ni) begin
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
    .req_i   (tag_cmd_req),
    .write_i (tag_cmd.write),
    .addr_i  (tag_cmd.addr),
    .wdata_i (tag_cmd.wdata),
    .wmask_i ('1),
    .rdata_o (tag_rdata),
    .cfg_i   ('0)
  );
endmodule
