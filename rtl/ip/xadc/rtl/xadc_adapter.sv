// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module xadc_adapter #(
  parameter  int Aw             = 10, // Width of TLUL address to expose
  parameter  int Dw             = 32, // Shall be matched with TL_DW
  localparam int Bw             = Dw/8

) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  output        drp_dclk_o,
  output        drp_den_o,
  output        drp_dwe_o,
  output  [6:0] drp_daddr_o,
  output [15:0] drp_di_o,
  input         drp_drdy_i,
  input  [15:0] drp_do_i

);

  // Signals to-from the SRAM interface of TLUL adapter
  logic          mem_req;
  logic          mem_gnt;
  logic          mem_we;
  logic [Aw-1:0] mem_addr;  // in bus words.
  logic [Dw-1:0] mem_wmask;
  logic [Dw-1:0] mem_wdata;
  logic          mem_rvalid;
  logic [Dw-1:0] mem_rdata;
  logic    [1:0] mem_rerror;

  // TLUL to SRAM adapter.
  // We can use the SRAM interface as an approximation of a DRP interface,
  // with a few tweaks.
  tlul_adapter_sram #(
    .SramAw(Aw),
    .SramDw(Dw),
    .Outstanding(1),
    .ByteAccess(0),
    .ErrOnWrite(0),
    .ErrOnRead(0),
    .CmdIntgCheck(0),
    .EnableRspIntgGen(1),
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

  // Conversion from byte addresses to 4-byte addresses is done
  assign drp_daddr_o = mem_addr;

  // Use only the bottom 16 bits of write data
  assign drp_di_o = mem_wdata[15:0];
  logic unused_wdata_bits;
  assign unused_wdata_bits = mem_wdata[Dw-1:16];

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
  logic above_addr_range, unaligned;
  // All accesses should be be 4-byte aligned.
  assign unaligned = |mem_addr[1:0];
  // All accesses should be to an address offset inside the
  // range of the 128 (0-127) XADC DRP registers.
  // 128 = 0x80; 0x80 << 2 = 0x200 = addr[9]; so check bit 9 and above.
  assign above_addr_range = |mem_addr[Aw-1:7];

  // Write-specific request issue detection
  logic wmask_bad, wr_to_rd_only_addr;
  // Writes should be to the lower 16-bits and no more in this simple
  // implementation, as the underlying XDC registers are all 16-bit registers.
  assign wmask_bad = mem_we && (~&mem_wmask[1:0] || |mem_wmask[Bw-1:2]);
  // Writes should be to an address offset inside the writable range.
  assign wr_to_rd_only_addr = mem_we && ~drp_daddr_o[6];

  // We have no way to reject bad requests, so we 'accept' and then drop them.
  // Read requests we can later respond to with an error,
  // but bad write requests we have to drop silently.
  logic drop_req;
  assign drop_req = unaligned || above_addr_range || wmask_bad || wr_to_rd_only_addr;
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
  assign mem_rdata[15:0] = mem_rerror[1] ? 'hDEADBEEF : drp_do_i;
  assign mem_rdata[Dw-1:16] = '0;


  // TODO figure out a way to add an ADC ID + version register


  `ASSERT_INIT(AddrWidthSufficientForDrp, Aw >= (7 + 2))

endmodule
