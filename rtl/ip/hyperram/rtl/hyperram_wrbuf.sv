// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Write buffering collects single-word write transactions to coalesce them
// into larger bursts when possible.
//
// Basic design rules:
//
// 1. Contiguous ascending writes (including bytes and half words) are collected
//    until we have reached the maximum burst length.
// 2. Up to two words of contiguous descending writes may be collected; this
//    limitation is because we have only a single word of internal storage.
// 3. Reads may overtake an under-construction burst write, with the proviso
//    that they cannot collide with the write data that is currently held.
// 4. Write data is NOT held indefinitely; a timer mechanism will flush out
//    the write data if the burst has not been extended with further data.
//    This is primarily to improve coherency with other ports of the HyperRAM,
//    but it also reduces the probability of subsequent read operations being
//    delayed.

module hyperram_wrbuf #(
  parameter int unsigned AW = 20,  // Width of address, bits.
  parameter int unsigned DW = 32,  // Width of data, bits.
  parameter int unsigned DBW = DW / 8,  // Number of write strobes.
  parameter int unsigned SeqWidth = 4,  // Width of sequence number, bits.
  parameter int unsigned Log2BurstLen = 5,  // Maximum of 32 bytes/burst.
  // Are reads permitted to overtake writes when there is no address collision?
  parameter bit ReadsOvertakeWrites = 1'b1,

  // LSB of word address.
  localparam int unsigned ABIT = $clog2(DW / 8)
) (
  input                         clk_i,
  input                         rst_ni,

  // Data to be written to the Downstream FIFO.
  output                        dfifo_wr_full_o,
  input               [DBW-1:0] dfifo_wr_strb_i,
  input                [DW-1:0] dfifo_wr_din_i,

  // Input command requests from the TL-UL port.
  input                         cmd_req_i,
  output                        cmd_wready_o,
  input             [AW-1:ABIT] cmd_mem_addr_i,
  input   [Log2BurstLen-ABIT:0] cmd_word_cnt_i,
  input                         cmd_wr_not_rd_i,
  input          [SeqWidth-1:0] cmd_seq_i,

  // Modified write traffic to the Downstream FIFO.
  output                        dfifo_wr_ena_o,
  input                         dfifo_wr_full_i,
  output              [DBW-1:0] dfifo_wr_strb_o,
  output               [DW-1:0] dfifo_wr_din_o,

  // Modified command requests to the HyperRAM controller.
  output                        cmd_req_o,
  input                         cmd_wready_i,
  output            [AW-1:ABIT] cmd_mem_addr_o,
  output  [Log2BurstLen-ABIT:0] cmd_word_cnt_o,
  output                        cmd_wr_not_rd_o,
  output                        cmd_wrap_not_incr_o,
  output         [SeqWidth-1:0] cmd_seq_o
);

// This may at some point become a full write buffer with the ability to coalesce
// a number of TL-UL write transactions that form a (nearly-)contiguous block of
// data but are received out of order.
//
// For now it addresses the simple cases that may be handled without the need for
// snooping of buffered write data. This provides significant performance benefit
// for the common case of contiguous ascending and - less so - descending word writes.
//
// A single word of write data is held internally before it is committed to the
// Downstream FIFO to the HBMC. Obviously data that has been committed cannot be
// retrieved/modified by this logic. Committing the data words to be the FIFO
// before determining the burst length and issuing the write command is only possible
// because _just one_ port (the LSU) implements write coalescing in this manner.

localparam int unsigned BBIT = Log2BurstLen;

// Number of bits in the write timeout counter.
localparam int unsigned TimerW = 5;

// Is there a write transaction in this cycle?
wire wr_req = cmd_req_i & cmd_wr_not_rd_i;
// How about a read?
wire rd_req = cmd_req_i & ~cmd_wr_not_rd_i;

// Retained burst details.
logic wr_stored;
logic [TimerW-1:0] wr_timer;
logic [AW-1:ABIT] base_addr_stored;
logic [Log2BurstLen-1-ABIT:0] burst_len_m1;  // Bus words minus 1.
// Expectations about next write transaction.
logic [AW-1:ABIT] exp_addr;

// Retained write strobes and data.
logic [DBW-1:0] strb_stored;
logic [DW-1:0]  data_stored;

// Address of the word immediately above the current transaction; we can use this to check
// contiguity for both ascending and descending accesses.
wire [AW-1:ABIT] next_addr = cmd_mem_addr_i + 'b1;

// Is the new write contiguously above the previous write?
wire contig_above = (cmd_mem_addr_i == exp_addr);
// How about descending?
// Note: we can only accept a word that precedes the start when we have seen just one earlier word.
wire contig_below = (next_addr == base_addr_stored) & ~|burst_len_m1;
// Same address as previous write transaction? e.g. partial writes. There shouldn't really be
// any repeated full word stores to a single address, but we can handle them inexpensively.
wire addr_repeated = (next_addr == exp_addr);
// Can we coalesce a write transaction with an under-construction burst write?
//
// Note: this considers only the current transaction type, address and strobes; further
// qualification with the burst length may be required in the use of `coalesce`.
wire coalesce = &{wr_req, contig_above | contig_below | addr_repeated};

// Increment the burst length; we also use this logic when writing out the `cmd_word_cnt` because
// that HyperRAM controller expects a 1-based value and it's preferable to perform the increment
// here, on the lower clock frequency.
wire [Log2BurstLen-ABIT:0] next_len = burst_len_m1 + 'b1;

// Command FIFO is preventing progress?
logic cmd_stalled;
// Downstream FIFO is preventing progress?
logic dfifo_stalled;
// Can state advance?
wire stalled = cmd_stalled | dfifo_stalled;

// Read collision with current write burst?
// Note: the burst writes are linear, not wrapping, so it does not suffice to assume that the upper
// address bits are the same for all words within the burst.
//
// Note: this is a bit conservative to make the check less expensive; we could compare against the
// present length of the under-construction write burst, but this inexpensive test will catch the
// vast majority of cases, allowing reads to proceed without needlessly terminating the write burst.
wire addr_collision = wr_stored &
                    ((cmd_mem_addr_i[AW-1:BBIT] == base_addr_stored[AW-1:BBIT]) ||
                     (cmd_mem_addr_i[AW-1:BBIT] == exp_addr[AW-1:BBIT]) ||
                     !ReadsOvertakeWrites);  // Treat all reads as collisions?
wire rd_collision = rd_req & addr_collision;

// By reordering a pair of word writes we can coalesce writes to descending addresses;
// to achieve longer burst writes in this case would require a LIFO implementation since the
// HBMC and HyperRAM accept only ascending bursts.
logic wr_descending_del;

// Maximum length burst; word count is maximum and the final word is complete.
wire burst_max = &{burst_len_m1, strb_stored};

// Flush out the current write burst because the present transaction cannot be combined with it;
// this requires writing to the Commmand FIFO, and being sure to do so no later than the final
// data word is written into the Downstream FIFO.
wire wr_timeout = ~|wr_timer;
wire flush_write = wr_stored & |{wr_req & !coalesce,  // Write cannot be combined.
                                 wr_descending_del,   // Can only coalesce two descending words.
                                 burst_max,           // Maximum burst length reached.
                                 rd_collision,        // Read may collide with write burst.
                                 wr_timeout};         // Write data too old.

// Store this write in anticipation of constructing a burst write?
wire wr_start = wr_req & (!wr_stored | flush_write);

always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
    wr_stored <= 1'b0;
  end else if ((wr_start | flush_write) & !stalled) begin
    // We can start a new burst collection in the same cycle as flushing out the current one.
    wr_stored <= wr_start;
  end
end

// Is this the second (and final) word of a descending write?
wire wr_descending = &{wr_stored, wr_req, contig_below};

// After we have spotted a write transaction to a descending address, we must be sure to write out
// the initial word and not continue collecting; otherwise data would be lost.
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) wr_descending_del <= 1'b0;
  else if (!stalled) wr_descending_del <= wr_descending;
end

// Most write transactions are stored (delayed by at least one cycle to see whether coalescing is
// possible). The exception is when we already have one word stored and the current write
// transaction immediately precedes it, so we must reorder the words when writing them into the
// Downstream FIFO.
wire wr_storing = wr_req & ~(wr_stored & contig_below);

// Do not hang onto write data indefinitely.
//
// This both prevents a subsequent read collision suffering additional delay and reduces the
// likelihood of coherency problems caused by writing code into the HyperRAM via the LSU Data port
// and then reading it back via the Instruction Fetch port.
always_ff @(posedge clk_i) begin
  if (!stalled) begin
    if (wr_start | wr_storing) wr_timer <= {TimerW{1'b1}};
    else if (wr_stored) wr_timer <= wr_timer - 'b1;
  end
end

// Burst properties; address and burst length tracking.
always_ff @(posedge clk_i) begin
  if (!stalled) begin
    if (wr_start) begin
      // First word of a new burst.
      base_addr_stored <= cmd_mem_addr_i;
      burst_len_m1 <= 'h0;
      // Expected address of next write transaction.
      exp_addr <= next_addr;
    end else if (wr_req) begin
      // Contiguous ascending burst is the most common case.
      if (contig_above) exp_addr <= next_addr;
      // We can coalesce only two words for a descending burst.
      if (contig_below) base_addr_stored <= cmd_mem_addr_i;
      // Increment the burst length when a new word-aligned address is observed.
      if (contig_above | contig_below) begin
        burst_len_m1 <= next_len[Log2BurstLen-1-ABIT:0];
      end
    end
  end
end

// We must stall if we need to send a command but the command FIFO is unavailable
// (this could be because it's unavailable or simply because we have not yet won arbitration).
assign cmd_stalled = cmd_req_o & !cmd_wready_i;

// We can store only a single word of strobes/data locally before writing into the Downstream FIFO.
// If we spot a descending write then we must reorder the two words, writing out the second word
// immediately and its predecessor in the next stall-free cycle.
wire dfifo_wr_ena = flush_write | wr_descending | &{wr_stored, wr_storing, !addr_repeated};
// We must stall the sender and our internal logic if we cannot proceed with a data write.
assign dfifo_stalled = dfifo_wr_ena & dfifo_wr_full_i;

// Merge the write strobes when we receive a subsequent partial write to the same address.
wire merge_strobes = addr_repeated & ~wr_start;

// Capturing of burst strobes/data. We support partial word writes, collecting the strobes and
// the data bytes.
always_ff @(posedge clk_i) begin
  if (wr_storing & !stalled) begin
    strb_stored <= dfifo_wr_strb_i | (strb_stored & {DBW{merge_strobes}});
    for (int unsigned b = 0; b < DBW; b++) begin
      if (dfifo_wr_strb_i[b]) begin
        data_stored[b*8 +: 8] <= dfifo_wr_din_i[b*8 +: 8];
      end
    end
  end
end

// Write data out to the Downstream FIFO. Usually we're writing the stored values, but in the
// event of a descending write, we must reorder the two writes.
assign dfifo_wr_ena_o  = dfifo_wr_ena & !cmd_stalled;
assign dfifo_wr_strb_o = wr_descending ? dfifo_wr_strb_i : strb_stored;
assign dfifo_wr_din_o  = wr_descending ? dfifo_wr_din_i  : data_stored;

// Modified command traffic to the HyperRAM controller.
assign cmd_req_o = (flush_write & !dfifo_wr_full_i) | (rd_req & ~addr_collision);
assign cmd_mem_addr_o = (flush_write & !wr_descending) ? base_addr_stored : cmd_mem_addr_i;
assign cmd_word_cnt_o = flush_write ? next_len : cmd_word_cnt_i;
assign cmd_wr_not_rd_o = flush_write;
// Writes are linear, reads are wrapping.
assign cmd_wrap_not_incr_o = !flush_write;
// Sequence number applies only to read requests; it is simply returned in the Upstream FIFO.
assign cmd_seq_o = cmd_seq_i;

// Stall the sender if we cannot accept the current transaction.
assign cmd_wready_o = ~|{rd_req & flush_write,  // Must complete the write before we can read.
                         cmd_req_i & stalled};  // Cannot proceed when stalled.
// If we need to write a data word but the Downstream FIFO is full that implies that a previous
// command is still emptying the FIFO and - if necessary - we will stall the sender using
// `cmd_wready_o.`
assign dfifo_wr_full_o = 1'b0;

endmodule

