// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// A port provides read access to the HyperRAM, and optionally write access too.
// It retains up to a full burst of read data and must maintain coherency with any write traffic
// in the event that writes are supported.
//
// An instruction port need not support write operations and does not require tag bits.
//
// Tag bits may be supported for only part of the mapped HyperRAM address range, in which case
// case an attempt to set a tag bit at an address that is outside that range will result in a
// TL-UL error being returned and the write will not occur.

module hbmc_tl_port import tlul_pkg::*; #(
  // Width of HyperRAM address, in bits.
  parameter int unsigned HyperRAMAddrW = 20,
  // Address width of the portion that can store capabilities, in bits.
  parameter int unsigned HyperRAMTagAddrW = 19,
  // log2(burst length in bytes)
  parameter int unsigned Log2BurstLen = 5,  // 32-byte bursts.
  parameter int unsigned NumBufs = 4,
  parameter int unsigned PortIDWidth = 1,
  parameter int unsigned Log2MaxBufs = 2,
  parameter int unsigned SeqWidth = 6,

  // Does this port need to support TileLink write operations?
  parameter bit SupportWrites = 1,
  // Coalesce write transfers into burst writes to the HBMC?
  parameter bit CoalesceWrites = 1,

  // Derived address bit parameters.
  localparam int unsigned ABIT = $clog2(top_pkg::TL_DW / 8),
  localparam int unsigned BBIT = Log2BurstLen
) (
  input                                   clk_i,
  input                                   rst_ni,

  // Constant indicating port number.
  input                 [PortIDWidth-1:0] portid_i,

  // TL-UL interface.
  input  tl_h2d_t                         tl_i,
  output tl_d2h_t                         tl_o,

  // Write notification input.
  input                                   wr_notify_i,
  input            [HyperRAMAddrW-1:ABIT] wr_notify_addr_i,
  input             [top_pkg::TL_DBW-1:0] wr_notify_mask_i,
  input              [top_pkg::TL_DW-1:0] wr_notify_data_i,

  // Write notification output.
  output logic                            wr_notify_o,
  output logic      [top_pkg::TL_DBW-1:0] wr_notify_mask_o,
  output logic       [top_pkg::TL_DW-1:0] wr_notify_data_o,
  output logic     [HyperRAMAddrW-1:ABIT] wr_notify_addr_o,

  // Command data to the HyperRAM controller; command, address and burst length
  output logic                            cmd_req_o,
  input                                   cmd_wready_i,
  output logic     [HyperRAMAddrW-1:ABIT] cmd_mem_addr_o,
  output logic      [Log2BurstLen-ABIT:0] cmd_word_cnt_o,
  output logic                            cmd_wr_not_rd_o,
  output logic                            cmd_wrap_not_incr_o,
  output logic             [SeqWidth-1:0] cmd_seq_o,

  output logic                            tag_cmd_req,
  output logic  [HyperRAMTagAddrW-1:ABIT] tag_cmd_mem_addr,
  output logic                            tag_cmd_wr_not_rd,
  output                                  tag_cmd_wcap,

  output logic                            dfifo_wr_ena_o,
  input                                   dfifo_wr_full_i,
  output            [top_pkg::TL_DBW-1:0] dfifo_wr_strb_o,
  output             [top_pkg::TL_DW-1:0] dfifo_wr_din_o,

  // Read data from the HyperRAM
  output                                  ufifo_rd_ena,
  input                                   ufifo_rd_empty,
  input              [top_pkg::TL_DW-1:0] ufifo_rd_dout,
  input                    [SeqWidth-1:0] ufifo_rd_seq,
  input                                   ufifo_rd_last,

  // Tag read data interface.
  output                                  tag_rdata_rready,
  input                                   tl_tag_bit
);

/*----------------------------------------------------------------------------------------------------------------------------*/

  logic tl_req_fifo_wready;
  logic tl_req_fifo_le1;
  logic wr_notify_match;
  logic dfifo_wr_full;
  logic rdbuf_matches;  // Address matches within the read buffer.
  logic rdbuf_valid;    // Valid data is available within the read buffer.
  logic cmd_wready;
  logic can_accept;
  logic rdbuf_hit;
  logic rdbuf_re;
  logic wr_err;
  logic wr_req;
  logic rd_req;
  logic issue;

  // We can accept an incoming TileLink transaction when we've got space in the hyperram, tag
  // and TileLink request FIFOs. If we're taking in a write transaction we also need space in the
  // downstream FIFO(dfifo) for the write data.
  //
  // If a read hits in the buffer but the data is not yet available, wait until it arrives
  // from the HyperRAM controller. This is indicated by the 'valid' bit becoming set for that data
  // word.
  //
  // Note: If a read hits in the RAM we wait until the TL request FIFO has at most a single entry
  // because we don't have a FIFO for the read data itself.

  assign can_accept = tl_req_fifo_wready &&
                     ((rd_req & rdbuf_valid & tl_req_fifo_le1) || (!rdbuf_hit && cmd_wready)) &&
                     (tl_i.a_opcode == Get || ~dfifo_wr_full) &
                     ~(wr_notify_i & wr_notify_match);

  // Return an error response for any capability write to an address that cannot support tags.
  wire untagged_addr = |(tl_i.a_address[HyperRAMAddrW:0] >> HyperRAMTagAddrW) &
                         tl_i.a_user.capability;

/*----------------------------------------------------------------------------------------------------------------------------*/

  // Valid read request?
  assign rd_req = tl_i.a_valid & (tl_i.a_opcode == Get);
  // Valid write request?
  assign wr_req = tl_i.a_valid & (tl_i.a_opcode == PutFullData || tl_i.a_opcode == PutPartialData) &
                 (SupportWrites & !untagged_addr);

/*----------------------------------------------------------------------------------------------------------------------------*/
  // Invalid write request?
  assign wr_err = tl_i.a_valid & (tl_i.a_opcode == PutFullData || tl_i.a_opcode == PutPartialData) &
                 (untagged_addr | !SupportWrites);

/*----------------------------------------------------------------------------------------------------------------------------*/
  if (SupportWrites) begin
    // Issue write notifications.
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        wr_notify_o <= 1'b0;
      end else begin
        // Notification of a write occurring on this port.
        wr_notify_o      <= wr_req & can_accept;
        // Address to which the write was performed.
        wr_notify_addr_o <= tl_i.a_address[HyperRAMAddrW-1:ABIT];
        // Mask specifying the sub-words being written.
        wr_notify_mask_o <= tl_i.a_mask;
        // Data being written.
        wr_notify_data_o <= tl_i.a_data;
      end
    end
  end else begin
    // Do not issue write notifications from this port.
    assign wr_notify_o      = 1'b0;
    assign wr_notify_addr_o = '0;
    assign wr_notify_mask_o = '0;
    assign wr_notify_data_o = '0;
  end

/*----------------------------------------------------------------------------------------------------------------------------*/

  logic [SeqWidth-1:0] rdbuf_seq;  // Sequence number of read buffer contents.
  logic [top_pkg::TL_DW-1:0] rdbuf_dout;

  // Invalidate the read buffer contents when a write occurs.
  //
  // Write notifications have the highest priority and must immediately update or invalidate
  // the contents of the read buffer in the event of a collision. `wr_notify_i` is asserted for a
  // single cycle.
  //
  // The read buffer informs of us when a write notification hits in the buffer, and any
  // simultaneous system bus transaction must then be delayed because the buffer is busy.
  wire rdbuf_invalidate = &{SupportWrites, wr_req, rdbuf_matches, ~rdbuf_valid};
  wire rdbuf_update     = &{SupportWrites, wr_req, rdbuf_valid};

  // Issue a new burst read if a read is performed outside of the current buffered address range.
  wire rdbuf_set   = rd_req & ~rdbuf_matches & issue;
  assign rdbuf_hit = rd_req &  rdbuf_matches;
  // Read data available and can issue the read in this cycle. The read buffer will return the
  // data in the following cycle.
  assign rdbuf_re  = &{rd_req, rdbuf_valid, issue};

  // Read buffer retains up to `NumBufs` burst(s) of data read from the HyperRAM for this port;
  // the data arrives incrementally and may be returned as soon as it becomes available.
  //
  // Hit tests are performed in parallel on both the address for the current TL-UL transaction
  // and any write notification. The `matches` outputs indicate an address hit on one of the
  // internal burst buffers, and the `valid` output indicates that that there is valid data
  // available for the specified word being addressed.
  hyperram_rdbuf #(
    .AW           (HyperRAMAddrW),
    .DW           (top_pkg::TL_DW),
    .DBW          (top_pkg::TL_DBW),
    .NumBufs      (NumBufs),
    .PortIDWidth  (PortIDWidth),
    .Log2MaxBufs  (Log2MaxBufs),
    .SeqWidth     (SeqWidth),
    .BBIT         (BBIT)
  ) u_readbuf(
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),

    // Constant indicating the port number.
    .portid_i         (portid_i),

    // Read/update hit test.
    .addr_i           (tl_i.a_address[HyperRAMAddrW-1:ABIT]),
    .mask_i           (tl_i.a_mask),
    .data_i           (tl_i.a_data),
    .matches_o        (rdbuf_matches),
    .valid_o          (rdbuf_valid),

    // Write notification test.
    .wr_notify_i      (wr_notify_i),
    .wr_notify_addr_i (wr_notify_addr_i[HyperRAMAddrW-1:ABIT]),
    .wr_notify_mask_i (wr_notify_mask_i),
    .wr_notify_data_i (wr_notify_data_i),
    .wr_matches_o     (wr_notify_match),

    // Control of buffer content.
    .invalidate_i     (rdbuf_invalidate),
    .update_i         (rdbuf_update),
    .set_i            (rdbuf_set),
    .seq_o            (rdbuf_seq),

    // Reading from buffer.
    .read_i           (rdbuf_re),
    .rdata_o          (rdbuf_dout),

    // Writing into buffer.
    .write_i          (ufifo_rd_ena),
    .wseq_i           (ufifo_rd_seq),
    .wdata_i          (ufifo_rd_dout)
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  localparam int unsigned TL_REQ_FIFO_DEPTH = 4;
  localparam int unsigned TLReqFifoDepthW = prim_util_pkg::vbits(TL_REQ_FIFO_DEPTH+1);

  // Verdict on the TL-UL request.
  typedef enum logic [1:0] {
    TLRspRdBuf,   // Return buffered read data.
    TLRspRdFetch, // Fetching read data from HBMC.
    TLRspWrOk,    // Valid write.
    TLRspWrErr    // Return error on write.
  } tl_rsp_type_e;

  // Description of a queued TL-UL request-response.
  typedef struct packed {
    // Metadata from inbound TileLink transactions that needs to be saved to produce the response.
    logic [top_pkg::TL_AIW-1:0] tl_source;
    logic [top_pkg::TL_SZW-1:0] tl_size;
    // Response to be returned.
    tl_rsp_type_e               rsp_type;
  } tl_req_info_t;

  tl_req_info_t tl_req_fifo_wdata, tl_req_fifo_rdata;

  logic tl_req_fifo_wvalid;
  logic tl_req_fifo_rvalid, tl_req_fifo_rready;
  logic [TLReqFifoDepthW-1:0] tl_req_fifo_depth;

  // Reads from the buffer are issued into the TL request FIFO only when it has at most a single
  // entry, because otherwise we would need additional storage for the read data.
  assign tl_req_fifo_le1 = ~|tl_req_fifo_depth[TLReqFifoDepthW-1:1];

  tl_d2h_t tl_o_int;

  // To be a contender in the arbitration among all ports, we need to express our intention
  // to write into the command buffer.
  wire cmd_req = tl_i.a_valid && tl_req_fifo_wready && !rdbuf_hit &&
                (tl_i.a_opcode == Get || (!dfifo_wr_full & !wr_err));

  assign issue = tl_i.a_valid & can_accept;

  // Logic for handling incoming TileLink requests
  logic cmd_wr_not_rd;
  logic dfifo_wr_ena;
  always_comb begin
    cmd_wr_not_rd      = (tl_i.a_opcode != Get);
    tag_cmd_req        = 1'b0;
    dfifo_wr_ena       = 1'b0;
    tl_req_fifo_wvalid = 1'b0;

    if (issue) begin
      // Write to the relevant FIFOs and indicate ready on TileLink A channel
      // - no tag request if the write was rejected.
      tag_cmd_req        = !wr_err;
      tl_req_fifo_wvalid = 1'b1;
      // Write into the downstream FIFO only for a valid write.
      dfifo_wr_ena       = wr_req;
    end
  end

  assign tag_cmd_wr_not_rd = cmd_wr_not_rd;
  assign tag_cmd_mem_addr = tl_i.a_address[HyperRAMTagAddrW-1:ABIT];

  // Decide on the type of response to be sent; encoded using an enumeration to reduce FIFO width.
  tl_rsp_type_e tl_cmd_rsp = (rd_req ? (rdbuf_valid ? TLRspRdBuf : TLRspRdFetch)
                                     : (wr_err      ? TLRspWrErr : TLRspWrOk));
  assign tl_req_fifo_wdata = '{
    tl_source     : tl_i.a_source,
    tl_size       : tl_i.a_size,
    rsp_type      : tl_cmd_rsp
  };

  // We decant the read data from the 'Upstream FIFO' into the read buffer as soon as possible,
  // both to prevent the FIFO from overflowing and to avoid holding up other read ports.
  //
  // Note that we must extract all of the data that we requested, even if it's no longer relevant
  // and shall ultimately be discarded, i.e. check only the port ID number here.
  assign ufifo_rd_ena = (ufifo_rd_seq[SeqWidth-1:SeqWidth-PortIDWidth] == portid_i) &
                        ~ufifo_rd_empty;

  // Track the reading of bursts from the HyperRAM controller.
  logic ufifo_rd_bursting;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ufifo_rd_bursting <= 1'b0;
    end else if (ufifo_rd_ena & (ufifo_rd_last | ~ufifo_rd_bursting))
      ufifo_rd_bursting <= !ufifo_rd_last;
  end

  // First word of data returned as part of a burst read operation.
  logic [top_pkg::TL_DW-1:0] ufifo_dout_first;
  assign ufifo_dout_first = ufifo_rd_dout[top_pkg::TL_DW-1:0];

  // Decode control signals from the response type.
  wire tl_rsp_wr_not_rd   = (tl_req_fifo_rdata.rsp_type == TLRspWrOk ||
                             tl_req_fifo_rdata.rsp_type == TLRspWrErr);
  wire tl_rsp_rd_buffered = (tl_req_fifo_rdata.rsp_type == TLRspRdBuf);

  // If the data from the read buffer is not accepted immediately by the host we must register it
  // to prevent it being invalidated by another read.
  logic rdata_valid_q;
  logic [top_pkg::TL_DW-1:0] rdata_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rdata_valid_q <= 1'b0;
    end else if (tl_o_int.d_valid) begin
      if (tl_i.d_ready) rdata_valid_q <= 1'b0;  // Response sent.
      else begin
        // Capture read data and keep it stable until it is accepted by the host.
        rdata_valid_q <= !tl_rsp_wr_not_rd;
        if (!rdata_valid_q) begin
          rdata_q <= tl_rsp_rd_buffered ? rdbuf_dout : ufifo_dout_first;
        end
      end
    end
  end

  // Logic for sending out TileLink responses.
  // - write responses may be sent as soon as the host is ready to accept them.
  // - read responses may be sent as soon as the first word of data is available; we employ
  //   wrapping bursts to ensure that the first word of the burst is the one being requested
  //   by the TileLink host.  
  always_comb begin
    tl_o_int           = '0;
    if (tl_req_fifo_rvalid) begin
      // We have an incoming request that needs a response
      if (tl_rsp_wr_not_rd) begin
        // If it's a write then return an immediate response (early response is reasonable as any
        // read that could observe the memory cannot occur until the write has actually happened)
        tl_o_int.d_valid   = 1'b1;
      end else begin
        // Otherwise wait until we have the first word of data to return.
        tl_o_int.d_valid   = |{ufifo_rd_ena & ~ufifo_rd_bursting,  // Initial word of burst read.
                               tl_rsp_rd_buffered,  // From read buffer.
                               rdata_valid_q};  // Holding read data stable until accepted.
      end
    end
    tl_o_int.d_error           = (tl_req_fifo_rdata.rsp_type == TLRspWrErr);
    tl_o_int.d_opcode          = tl_rsp_wr_not_rd ? AccessAck : AccessAckData;
    tl_o_int.d_size            = tl_req_fifo_rdata.tl_size;
    tl_o_int.d_source          = tl_req_fifo_rdata.tl_source;
    tl_o_int.d_data            = rdata_valid_q ? rdata_q :
                                (tl_rsp_rd_buffered ? rdbuf_dout : ufifo_dout_first);
    tl_o_int.d_user.capability = tl_tag_bit;
    tl_o_int.a_ready           = issue;
  end

  // Complete the TL request as soon the response is accepted; this avoids the need to register
  // the properties of the response.
  assign tl_req_fifo_rready = tl_o_int.d_valid & tl_i.d_ready;

  // Discard the tag read data once the _read_ data is accepted.
  assign tag_rdata_rready = tl_o_int.d_valid & tl_i.d_ready & ~tl_rsp_wr_not_rd;

  // Generate integrity for outgoing response.
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(0),
    .EnableDataIntgGen(0)
  ) u_tlul_rsp_intg_gen (
    .tl_i (tl_o_int),
    .tl_o (tl_o)
  );

  // Queue of pending TileLink requests.
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
    .depth_o (tl_req_fifo_depth),
    .err_o   ()
  );

  // Command requests to the HyperRAM controller.
  //
  // If this port performs write coalescing, these commands may be modified or suppressed by the
  // `hyperram_wrbuf` instance below.
  wire [HyperRAMAddrW-1:ABIT] cmd_mem_addr = tl_i.a_address[HyperRAMAddrW-1:ABIT];
  wire [Log2BurstLen-ABIT:0]  cmd_rd_len   = {1'b1, {(Log2BurstLen-ABIT){1'b0}}};  // Full burst.
  wire [Log2BurstLen-ABIT:0]  cmd_wr_len   = {{(Log2BurstLen-ABIT){1'b0}}, 1'b1};  // Single word.
  wire [Log2BurstLen-ABIT:0]  cmd_word_cnt = (tl_i.a_opcode == Get) ? cmd_rd_len : cmd_wr_len;
  // Write bursts are linear, reads wrap. Linear bursts are more beneficial to write coalescing,
  // but read bursts are wrapping, so that the requested data may be returned as soon as possible.
  wire cmd_wrap_not_incr = (tl_i.a_opcode == Get);
  wire [SeqWidth-1:0] cmd_seq = rdbuf_seq;

  assign tag_cmd_wcap = tl_i.a_user.capability;

  // Write buffer performs basic write coalescing to produce larger write bursts.
  if (SupportWrites && CoalesceWrites) begin : gen_write_buffer
    // This logic sits between the TL-UL handling and the Command and Downstream
    // Data FIFOs, modifying the traffic.
    // It must also be aware of HyperRAM reads in order to flush out any buffered
    // write data first in the event of a collision.
    hyperram_wrbuf #(
      .AW           (HyperRAMAddrW),
      .DW           (top_pkg::TL_DW),
      .DBW          (top_pkg::TL_DBW),
      .Log2BurstLen (Log2BurstLen),
      .SeqWidth     (SeqWidth)
    ) u_writebuf(
      .clk_i                (clk_i),
      .rst_ni               (rst_ni),

      // Data to be written into the Downstream FIFO.
      .dfifo_wr_full_o      (dfifo_wr_full),
      .dfifo_wr_strb_i      (tl_i.a_mask),
      .dfifo_wr_din_i       (tl_i.a_data),

      // Input command requests for any TL-UL operation that could not be fully
      // satisfied by the read buffer.
      .cmd_req_i            (cmd_req),
      .cmd_wready_o         (cmd_wready),
      .cmd_mem_addr_i       (cmd_mem_addr),
      .cmd_word_cnt_i       (cmd_word_cnt),
      .cmd_wr_not_rd_i      (cmd_wr_not_rd),
      .cmd_seq_i            (cmd_seq),

      // Modified write traffic to the Downstream FIFO.
      .dfifo_wr_ena_o       (dfifo_wr_ena_o),
      .dfifo_wr_full_i      (dfifo_wr_full_i),
      .dfifo_wr_strb_o      (dfifo_wr_strb_o),
      .dfifo_wr_din_o       (dfifo_wr_din_o),

      // Modified command requests to the HyperRAM controller.
      .cmd_req_o            (cmd_req_o),
      .cmd_wready_i         (cmd_wready_i),
      .cmd_mem_addr_o       (cmd_mem_addr_o),
      .cmd_word_cnt_o       (cmd_word_cnt_o),
      .cmd_wr_not_rd_o      (cmd_wr_not_rd_o),
      .cmd_wrap_not_incr_o  (cmd_wrap_not_incr_o),
      .cmd_seq_o            (cmd_seq_o)
    );

    // Write buffer logic controls the Downstream FIFO.
    logic unused_wrbuf;
    assign unused_wrbuf = ^{dfifo_wr_ena, cmd_wrap_not_incr};
  end else begin : gen_no_write_buffer
    // Commands to the HyperRAM controller propagate unmodified.
    assign cmd_req_o           = cmd_req;
    assign cmd_wready          = cmd_wready_i;
    assign cmd_mem_addr_o      = cmd_mem_addr;
    assign cmd_word_cnt_o      = cmd_word_cnt;
    assign cmd_wr_not_rd_o     = cmd_wr_not_rd;
    assign cmd_wrap_not_incr_o = cmd_wrap_not_incr;
    assign cmd_seq_o           = cmd_seq;

    // Data to be written into the Downstream FIFO.
    assign dfifo_wr_ena_o  = dfifo_wr_ena;
    assign dfifo_wr_full   = dfifo_wr_full_i;
    assign dfifo_wr_strb_o = tl_i.a_mask;
    assign dfifo_wr_din_o  = tl_i.a_data;
  end

  // Unused signals.
  logic unused;
  assign unused = ^{tl_i.a_param, tl_req_fifo_depth[0]};

endmodule

