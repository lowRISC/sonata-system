// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module xadc_adapter #(
  parameter int TAw = 10, // Width of TLUL address to expose
  parameter int TDw = 32, // Shall be matched with TL_DW
  parameter int XAw = 7,  // XADC DRP address width (for readability)
  parameter int XDw = 16  // XADC DRP data width (for readability)
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  output           drp_dclk_o,
  output           drp_den_o,
  output           drp_dwe_o,
  output [XAw-1:0] drp_daddr_o,
  output [XDw-1:0] drp_di_o,
  input            drp_drdy_i,
  input  [XDw-1:0] drp_do_i
);

  // Signals to-from the SRAM interface of TLUL adapter
  logic           mem_req;
  logic           mem_gnt;
  logic           mem_we;
  logic [TAw-1:0] mem_addr;  // in bus words.
  logic [TDw-1:0] mem_wmask;
  logic [TDw-1:0] mem_wdata;
  logic           mem_rvalid;
  logic [TDw-1:0] mem_rdata;
  logic     [1:0] mem_rerror;

  // TLUL to SRAM adapter.
  // We can use the SRAM interface as an approximation of a DRP interface,
  // with a few tweaks.
  tlul_adapter_sram #(
    .SramAw(TAw),
    .SramDw(TDw),
    .Outstanding(1),
    .ByteAccess(0),
    .ErrOnWrite(0),
    .ErrOnRead(0),
    .CmdIntgCheck(0),
    .EnableRspIntgGen(0),
    .EnableDataIntgGen(0),
    .EnableDataIntgPt(0),
    .SecFifoPtr(0)
  ) u_tlul_adapter (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),

    .tl_i        (tl_i),
    .tl_o        (tl_o),

    .en_ifetch_i (prim_mubi_pkg::MuBi4False),

    .req_o       (mem_req),
    .req_type_o  (),
    .gnt_i       (mem_gnt),
    .we_o        (mem_we),
    .addr_o      (mem_addr),
    .wdata_o     (mem_wdata),
    .wdata_cap_o (),
    .wmask_o     (mem_wmask),
    .intg_error_o(),
    .rdata_i     (mem_rdata),
    .rdata_cap_i (1'b0),
    .rvalid_i    (mem_rvalid),
    .rerror_i    (mem_rerror)
  );

  // Use the supplied clock as the XADC DCLK.
  // This is divided internally to generate ADCCLK.
  assign drp_dclk_o = clk_i;

  assign drp_dwe_o = mem_we;

  // Conversion from TLUL byte addresses to DRP register addresses
  // is done in tlul_adapter_sram.
  assign drp_daddr_o = mem_addr[XAw-1:0];

  // Use only the bottom 16 bits of write data
  assign drp_di_o = mem_wdata[XDw-1:0];
  logic unused_wdata_bits;
  assign unused_wdata_bits = ^mem_wdata[TDw-1:XDw];

  // The DRP interface can process only one request at a time and can take
  // some cycles to fulfil a request. So we need to track whether we have
  // an outstanding read/write request the DRP is still working on.
  logic drp_busy_d;
  logic drp_busy_q;
  assign drp_busy_d = drp_den_o || (drp_busy_q && ~drp_drdy_i);
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      drp_busy_q <= '0;
    end else begin
      drp_busy_q <= drp_busy_d;
    end
  end

  // General request issue detection
  logic above_addr_range;
  // All accesses should be to an address offset inside the
  // range of the 128 (0-127) XADC DRP registers.
  assign above_addr_range = |mem_addr[TAw-1:XAw];

  // Write-specific request issue detection
  logic wmask_bad, wr_to_rd_only_addr;
  logic unused_wmask_bits;
  // Writes should include the lower 16-bits,
  // as the underlying XDC registers are all 16-bit registers.
  assign wmask_bad = mem_we && ~&mem_wmask[XDw-1:0];
  assign unused_wmask_bits = ^mem_wmask[TDw-1:XDw];
  // Writes should be to an address offset inside the writable range.
  assign wr_to_rd_only_addr = mem_we && ~drp_daddr_o[XAw-1];

  // We have no way to reject bad requests, so we 'accept' and then drop them.
  // Read requests we can later respond to with an error,
  // but bad write requests we have to drop silently.
  logic drop_req;
  assign drop_req = above_addr_range || wmask_bad || wr_to_rd_only_addr;
  // Grant host requests if the DRP is ready or we are just going to drop it.
  assign mem_gnt = mem_req && (~drp_busy_q || drop_req);

  // Only pass valid requests through to the DRP
  assign drp_den_o = mem_gnt && ~drop_req;

  // Break the return path timing by reporting bad read requests
  // the clock cycle *after* the request was 'granted'.
  logic rd_err_d;
  logic rd_err_q;
  assign rd_err_d = mem_gnt && ~mem_we && drop_req;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rd_err_q <= '0;
    end else begin
      rd_err_q <= rd_err_d;
    end
  end

  assign mem_rvalid = drp_drdy_i || rd_err_q;

  assign mem_rerror[0] = 1'b0; // uninteresting "correctable error" bit
  assign mem_rerror[1] = rd_err_q; // "uncorrectable error" bit

  // The host should know better than to use rdata if rerror[1] is set,
  // but set rdata to a memorable 'error' value just in case.
  assign mem_rdata = mem_rerror[1] ? 'hDEADBEEF : {{(TDw-XDw){1'b0}},drp_do_i};


endmodule
