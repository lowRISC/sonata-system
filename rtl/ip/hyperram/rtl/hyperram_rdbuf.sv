// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Read buffer retains the contents of a read burst, on the premise that subsequent words within
// the burst will be required by the CPU in the near future.
module hyperram_rdbuf #(
  // System bus side.
  parameter int unsigned AW = 20, // Width of address, bits.
  parameter int unsigned DW = 32, // Width of data, bits.
  parameter int unsigned DBW = 4,  // Number of update strobes.
  parameter int unsigned NumBufs = 4,  // Number of read buffers.
  parameter int unsigned PortIDWidth = 1, // Width of Port ID, bits.
  parameter int unsigned Log2MaxBufs = 2,
  parameter int unsigned SeqWidth = 6, // Width of sequence number, bits.
  // Burst size of 32 bytes
  parameter int unsigned BBIT = 5,  // 32 bytes/burst.

  // LSB of word address.
  localparam int unsigned ABIT = $clog2(DW / 8),
  // Size of read buffer in words of 'DW' bits.
  localparam int unsigned BufWords = 1 << (BBIT - ABIT)
) (
  input                       clk_i,
  input                       rst_ni,

  // Constant indicating the port number.
  input     [PortIDWidth-1:0] portid_i,

  // Hit test for read/update access.
  input           [AW-1:ABIT] addr_i,
  input             [DBW-1:0] mask_i,
  input              [DW-1:0] data_i,
  output                      matches_o,
  output                      valid_o,

  // Write notification test.
  input                       wr_notify_i,
  input           [AW-1:ABIT] wr_notify_addr_i,
  input             [DBW-1:0] wr_notify_mask_i,
  input              [DW-1:0] wr_notify_data_i,
  output                      wr_matches_o,

  // Control of buffer content.
  input                       invalidate_i,
  input                       update_i,
  input                       set_i,
  output logic [SeqWidth-1:0] seq_o,

  // Reading from the buffer (System bus side).
  input                       read_i,
  output logic       [DW-1:0] rdata_o,

  // Writing to the buffer (HyperRAM side).
  input                       write_i,
  input        [SeqWidth-1:0] wseq_i,
  input              [DW-1:0] wdata_i
);

// Round robin replacement of read buffers when all are occupied.
//
// TODO: Investigate any benefit from LRU instead? Since there are presently just 4 buffers
// per port, it is simple and inexpensive to keep a rank number for each of the buffers.
logic [NumBufs-1:0] buf_replace;
if (NumBufs > 1) begin : gen_round_robin_replace
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) buf_replace <= 'b1;
    else if (set_i) buf_replace <= {buf_replace[NumBufs-2:0], buf_replace[NumBufs-1]};
  end
end else begin
  assign buf_replace = 1'b0;
end

logic [NumBufs-1:0] configured;

// Decide upon the buffer to be replaced; if at least one is available, arbitrarily pick the
// lowest-numbered available buffer. If none is available then we use round-robin replacement.
wire [NumBufs-1:0] avail_lsb = ~configured & ~(~configured - 'b1);
wire [NumBufs-1:0] buf_set = &configured ? buf_replace : avail_lsb;

// Validity bits for buffer words.
logic [NumBufs-1:0][BufWords-1:0] valid;

// Base address of buffer contents.
logic [NumBufs-1:0][AW-1:BBIT] base_addr;

// Individual responses from hit tests on the buffers.
logic [NumBufs-1:0] matches_all;
logic [NumBufs-1:0] valid_all;
logic [NumBufs-1:0] wr_matches_all;
logic [NumBufs-1:0] wr_valid_all;

// Combined response.
assign matches_o    = |matches_all;
assign valid_o      = |valid_all;
assign wr_matches_o = |wr_matches_all;

// A write notification that hits must be serviced immediately and with highest priority; it is
// a single-cycle event that either invalidates the affected buffer or updates its contents.
//
// The parent module is informed of the hit, so that it can hold off any system bus transaction
// occurring in the same cycle, but it does not need to know what action was taken.
//
// Internally we either invalidate the buffer, if there is no valid data available, or we perform
// an update of the buffer contents.
wire wr_valid = |wr_valid_all;
wire wr_notify_invalidate = wr_notify_i & wr_matches_o & ~wr_valid;
wire wr_notify_update     = wr_notify_i & wr_matches_o &  wr_valid;

// Invalidate occurs when matching but not valid, and we need to know which valid signal to consult.
wire [NumBufs-1:0] invalidate = ({NumBufs{invalidate_i}} & matches_all) |
                                ({NumBufs{wr_notify_invalidate}} & wr_matches_all);
wire [NumBufs-1:0] set = {NumBufs{set_i}} & buf_set;

// Hit test on buffer contents.
logic [BBIT-1:ABIT] a_offset;
assign a_offset = addr_i[BBIT-1:ABIT];  // Address bits selecting word within burst.

// Return the bit index of the single bit set within the input; the output is undefined in the
// event of zero bits or more than one bit being set and shall not be used.
// This could only happen as a result of a design fault; it implies multiple buffers holding
// data for the same address.
function automatic logic [Log2MaxBufs-1:0] one_hot_enc(logic [NumBufs-1:0] in);
  logic [Log2MaxBufs-1:0] out = 0;
  for (int unsigned b = 0; b < Log2MaxBufs; b++) begin
    for (int unsigned i = 0; i < NumBufs; i++) out[b] = out[b] | (in[i] & i[b]);
  end
  return out;
endfunction

// Offset of write notification address within burst.
logic [BBIT-1:ABIT] wn_offset;
assign wn_offset = wr_notify_addr_i[BBIT-1:ABIT];  // Address bits selecting word within burst.

// Offset of read or update from within burst; this is either a write notification that hits
// (top priority) or a write transaction that hits.
logic [BBIT-1:ABIT] ur_offset;
assign ur_offset = wr_notify_update ? wn_offset : a_offset;

// Buffer to be read by the system bus, or updated by the system bus or write notification.
wire [NumBufs-1:0] ur_valid_all = wr_notify_update ? (wr_matches_all & wr_valid_all) : valid_all;
wire [Log2MaxBufs-1:0] ur_buf = one_hot_enc(ur_valid_all);

// Updating of buffer contents.
wire           update = update_i | wr_notify_update;
wire [DBW-1:0] umask  = wr_notify_update ? wr_notify_mask_i : mask_i;
wire [DW-1:0]  udata  = wr_notify_update ? wr_notify_data_i : data_i;

// When receiving data from the HyperRAM controller, it arrives tagged with the buffer number.
localparam int unsigned SeqBits = SeqWidth - PortIDWidth - Log2MaxBufs;
logic [Log2MaxBufs-1:0] wr_buf = wseq_i[SeqBits +: Log2MaxBufs];
logic [NumBufs-1:0] wr_accepted;

// Offset at which the next word of returned data shall be written, for each buffer;
// each buffer may have a single outstanding and still current request.
logic [NumBufs-1:0][BBIT-1:ABIT] woffset;

// Sequence number is driven to non-zero only by the buffer which is about to be (re-)filled,
// so we can just OR all of the sequence numbers together.
logic [NumBufs-1:0][SeqWidth-1:0] seq_all;
always_comb begin
  seq_o = 0;
  for (int unsigned b = 0; b < NumBufs; b++) seq_o = seq_o | seq_all[b];
end

// The read buffer is capable of retaining a number of bursts of read data, so some of the
// control logic - chiefly the address matching - must be replicated for each of these
// internal buffers.
for (genvar b = 0; b < NumBufs; b++) begin : gen_buf_state
  // Does the buffer have valid information?
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      configured[b] <= 1'b0;
    end else if (invalidate[b] | set[b]) begin
      configured[b] <= set[b];
    end
  end

  // Updating of validity bits.
  always_ff @(posedge clk_i) begin
    // System bus-side changes in the buffer status take precedence over newly-received read data.
    if (invalidate[b] | set[b]) begin
      valid[b] <= 'b0;
    end else if (wr_accepted[b]) begin
      valid[b][woffset[b]] <= 1'b1;
    end
  end

  // Indicates that the R/W address matches within this read buffer.
  assign matches_all[b] = &{configured[b], addr_i[AW-1:BBIT] == base_addr[b][AW-1:BBIT]};
  // Since these validity indicators are combined and returned to the parent as a single indication,
  // we must qualify it here with `matches.`
  assign valid_all[b] = matches_all[b] & valid[b][a_offset];

  // Write notification test; this is done in parallel with normal read buffer access because
  // normally writes on other TL-UL ports will not collide with buffered read data.
  assign wr_matches_all[b] = &{configured[b],
                               wr_notify_addr_i[AW-1:BBIT] == base_addr[b][AW-1:BBIT]};
  // Since this `validity` indicator is used only internally there is no need to qualify it here
  // with `matches.`
  assign wr_valid_all[b] = valid[b][wn_offset];

  // Sequence number for the buffer contents.
  logic [2:0] next_seq;
  logic [2:0] seq;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) seq <= '0;
    else if (set[b]) seq <= next_seq;
  end
  // A single bit suffices for the sequence number.
  assign next_seq = seq + 'b1;
  // When issuing a new burst read, it's the new sequence number that is required, so that we
  // accept the returned data. Drive our sequence number to zero if we're not just starting
  // a fill request, so that all sequence numbers can simply be ORed together.
  assign seq_all[b] = {SeqWidth{set[b]}} & {portid_i, b[Log2MaxBufs-1:0], next_seq};

  // Base address of buffer contents.
  always_ff @(posedge clk_i) begin
    if (set[b]) base_addr[b] <= addr_i[AW-1:BBIT];
  end

  // Writes are accepted only if the buffer is still configured and the write data belongs in
  // the currently-buffered content. The port ID number has already been checked; only the
  // buffer number and the sequence number matter here.
  assign wr_accepted[b] = &{write_i, configured[wr_buf], wr_buf == b,
                           (wseq_i[SeqWidth-1-PortIDWidth-Log2MaxBufs:0] == seq)};

  // Writing into the buffer.
  always_ff @(posedge clk_i) begin
    if (set[b]) begin
      // Retain the offset of the first word that will be returned by the wrapping burst.
      woffset[b] <= addr_i[BBIT-1:ABIT];
    end else begin
      // Wrapping bursts are achieved by `woffset` overflowing at the end of the burst.
      woffset[b] <= woffset[b] + {{(BBIT-ABIT-1){1'b0}}, wr_accepted[b]};
    end
  end
end

// RAM submodule wants a single write strobe per data line.
localparam int unsigned DataBitsPerMask = DW / DBW;
logic [DW-1:0] umask_full;
always_comb begin
  for (int unsigned b = 0; b < DBW; b++) begin
    umask_full[b*DataBitsPerMask +: DataBitsPerMask] = {DataBitsPerMask{umask[b]}};
  end
end

// Use a dual-port implementation for simplicity because the design is targeting an FPGA
// implementation. Read-write collisions will be infrequent but we DO need to handle them.
prim_ram_2p #(
  .Width            (DW),
  .Depth            (NumBufs * BufWords),
  .DataBitsPerMask  (DataBitsPerMask)
) u_buf(
  .clk_a_i      (clk_i),
  .clk_b_i      (clk_i),

  // Read/update port (TL-UL side).
  // - update and read shall not occur simultaneously; let update take precedence.
  .a_req_i      (update | read_i),
  .a_write_i    (update),
  .a_addr_i     ({ur_buf, ur_offset}),
  .a_wdata_i    (udata),
  .a_wmask_i    (umask_full),
  .a_rdata_o    (rdata_o),

  // Write port (HyperRAM side).
  .b_req_i      (write_i),
  .b_write_i    (1'b1),
  .b_addr_i     ({wr_buf, woffset[wr_buf]}),
  .b_wdata_i    (wdata_i),
  .b_wmask_i    ('1),
  .b_rdata_o    (),  // Write-only port.

  .cfg_i        ('0)
);

logic unused;
assign unused = ^wseq_i;  // The port number has already been checked in the parent.

endmodule

