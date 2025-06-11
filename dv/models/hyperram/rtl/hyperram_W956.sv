// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a basic functional model of a HyperRAM device for simulation,
// based on the W956D8MBYA.
//
// Presently it has the following limitations:
// - It does not model internal refresh operations, so it has a fixed access latency.
// - It does not drive RWDS during the command-address interval, but since Verilator
//   holds the signal low in this case, the behaviour is as intended.
// - Register reading is not implemented; the HyperBus controller being used does
//   not issue register reads as it stands.
//
// See the W956D8MBYA datasheet for details. The following is a quick sketch to
// aid understanding. In particular, note that the clock - which is differential to
// support DDR - is received only when a transfer is occurring:
//     _______                                                           ___________
// csn        \_________________________________________________________/
//                ___     ___     ___     ___     ___     ___     ___
// ckp __________|   \___|   \___|   \___|   \___|   \___|   \___|   \______________
//     __________     ___     ___     ___     ___     ___     ___     ______________
// ckn           \___|   \___|   \___|   \___|   \___|   \___|   \___|
//               __  __  __  __  __  __  __  __
// rwds/ _______|  \|  \|  \|  \|  \|  \|  \|  \
// dq           \__|\__|\__|\__|\__|\__|\__|\__|
//
// Both the read/write strobes (rwds) and data (dq) are sampled on both clock edges.
// Note that data is transferred as 16-bit Big Endian words.

module hyperram_W956(
  // Asynchronous reset.
  input       rstn,
  // Differential clocking (DDR).
  input       ckp,
  input       ckn,
  // Chip Select.
  input       csn,
  // Bidirectional read/write data strobe.
  inout       rwds,
  // Bidirectional data bus.
  inout [7:0] dq
);
  // Report memory and register traffic occurring within the HyperRAM model?
  localparam bit HyperRAM_Logging = 1'b0;

  // HyperRAM storage.
  logic [21:0] hram_addr;
  logic [15:0] hram_wdata;
  logic [15:0] hram_wmask;
  logic [15:0] hram_rdata;
  logic        hram_we;
  logic [1:0]  hram_be;

  // Expand the write strobes to bit level as required by the memory model.
  assign hram_wmask = {{8{hram_be[1]}}, {8{hram_be[0]}}};

  prim_ram_2p #(
    .Width           (16),
    .Depth           ('h40_0000),
    .DataBitsPerMask (8),
    .MemInitFile     ()
  ) u_ram(
    // Reads are performed on the rising edge of the positive clock (CK)
    // and the read data is returned combinationally.
    .clk_a_i    (ckp),
    // Writes are performed on the rising edge of the negative clock (CK#)
    // once the entire 16-bit word has been received. Note that there is not
    // necessarily another clock edge after that because CK/CK# are static
    // when not transferring data over the HyperBus.
    .clk_b_i    (ckn),

    // Read port.
    .a_req_i    (hram_re),
    .a_write_i  (1'b0),
    .a_addr_i   (hram_addr),
    .a_wdata_i  ('0),
    .a_wmask_i  ('0),
    .a_rdata_o  (hram_rdata),

    // Write port.
    .b_req_i    (hram_we),
    .b_write_i  (1'b1),
    .b_addr_i   (hram_addr),
    .b_wdata_i  (hram_wdata),
    .b_wmask_i  (hram_wmask),
    .b_rdata_o  (),

    .cfg_i      ('0)
   );

  // HyperRAM model; this is a very basic model, just enough to allow us to simulate
  // actual usage. Note that the timing is not completely accurate because there is
  // no effort to simulate the variable latency that results from internal refresh
  // activity.
  typedef enum logic [3:0] {
    HRAM_Cmd0,
    HRAM_Cmd1,
    HRAM_Cmd2,
    HRAM_Cmd3,
    HRAM_Cmd4,
    HRAM_Cmd5,
    HRAM_Latency1,
    HRAM_Latency2,
    HRAM_Writing,  // Actively collecting write data until CS raised.
    HRAM_Reading,  // Actively returning read data until CS raised.
    HRAM_RegWriting,  // Two cycles of register write data.
    // The HyperBus Memory Controller that we use does not issues register reads.
    HRAM_Ignoring  // Ignoring register read.
  } hram_state_e;

  logic hram_re;
  assign hram_we = &{hram_state == HRAM_Writing, !csn};
  assign hram_re = &{hram_state == HRAM_Reading, !csn};

  hram_state_e hram_state;
  logic [2:0] hram_cmd;
  logic [4:0] hram_lat;
  // High byte of current 16-bit write operation.
  logic [7:0] hram_wdata_hi;
  // Write strobe for high byte.
  logic hram_wmask_hi;
  logic hram_lsb;  // LSByte within the current word?

  // Write data and strobes.
  assign hram_be[1] = hram_wmask_hi;
  assign hram_be[0] = !rwds;
  assign hram_wdata = {hram_wdata_hi, dq};

  // Read data path.
  logic hram_drv_oe;
  logic hram_rd_sel;
  logic hram_rd_lsb;
  // Selected byte; MS byte is returned first.
  wire [7:0] hram_rd_byte = hram_rd_lsb ? hram_rdata[7:0] : hram_rdata[15:8];

  // Burst wrapping.
  logic burst_end;
  logic burst_wrap;
  always_comb begin
    burst_end = &{hram_addr[5:0], hram_lsb};  // 128 bytes
    case (burst_len)
      2'b01:   burst_end = &{hram_addr[4:0], hram_lsb};  // 64 bytes
      2'b10:   burst_end = &{hram_addr[2:0], hram_lsb};  // 16 bytes
      2'b11:   burst_end = &{hram_addr[3:0], hram_lsb};  // 32 bytes
      default: /* see default assignment above */;
    endcase
  end
  logic [21:0] hram_base;
  always_comb begin
    hram_base = {hram_addr[21:6], 6'b0};
    case (burst_len)
      2'b01:   hram_base = {hram_addr[21:5], 5'b0};
      2'b10:   hram_base = {hram_addr[21:3], 3'b0};
      2'b11:   hram_base = {hram_addr[21:4], 4'b0};
      default: /* see default assignment above */;
    endcase
  end
  assign burst_wrap = !hram_cmd[0];  // Wrapping, rather than linear burst.

  // Address for next transfer; bursts may be wrapping or linear. A linear burst just
  // continues incrementing the address indefinitely.
  logic [21:0] next_addr;
  assign next_addr = (burst_end & burst_wrap) ? hram_base : (hram_addr + 22'(hram_lsb));

  // Configuration Register 0 state.
  logic [3:0] latency_initial;
  logic       latency_fixed;
  logic       burst_hybrid;  // Note: Hybrid burst mode is not required at present.
  logic [1:0] burst_len;

  // Configuration Register 1 state.
  //
  // Note: We currently don't model this register/functionality; it is concerned with sleep mode
  // and refresh functionality.

  // Configuration register write.
  wire hram_cfg_we = &{hram_state == HRAM_RegWriting, !csn};
  // Note: We do not implement register reading because the HBMC does not use it.

  // We need only be concerned with Configuration Register 0 and Configuration Register 1 writes,
  // and chiefly to ignore writes to register 1 which we presently do not model.
  wire hram_cfg_reg = hram_addr[0];
  wire [15:0] hram_cfg_wdata = {hram_wdata_hi, dq};

  always_ff @(posedge ckn, negedge rstn) begin
    if (!rstn) begin
      // Configuration Register 0.
      latency_initial <= 4'b0010;  // Default 7 Clock Latency @ 200MHz.
      latency_fixed <= 1'b1;  // Default is 2 times Initial Latency.
      burst_hybrid <= 1'b1;  // Legacy wrapped bursts are the default.
      burst_len <= 2'b11;  // Default to 32-byte bursts when wrapping.
    end else if (hram_cfg_we & ~hram_cfg_reg) begin
      latency_initial <= hram_cfg_wdata[7:4];
      latency_fixed <= hram_cfg_wdata[3];
      burst_hybrid <= hram_cfg_wdata[2];
      burst_len <= hram_cfg_wdata[1:0];
    end
  end

  // Logging of RAM and Register activity.
  if (HyperRAM_Logging) begin : logging
    // Memory traffic.
    logic [21:0] hram_addr_del;
    logic hram_re_del;
    always_ff @(posedge ckn) begin : logging_mem
      hram_re_del <= hram_re;
      hram_addr_del <= hram_addr;
      // Writes occur on the rising edge of ckn, so the write inputs are available at this point.
      if (hram_we) begin
        $display("%t: HR write 0x%0x : %02b : 0x%04x", $realtime, hram_addr, hram_be, hram_wdata);
      end
      // Reads occur on the rising edge of ckp, but we want to report the new state of `hram_rdata.`
      if (hram_re_del) begin
        $display("%t: HR read 0x%0x -> 0x%04x", $realtime, hram_addr_del, hram_rdata);
      end
    end : logging_mem
    // Register traffic.
    always_ff @(posedge ckn) begin : logging_reg
      if (hram_cfg_we) begin
        $display("%t: HR reg write 0x%0x : 0x%04x", $realtime, hram_cfg_reg, hram_cfg_wdata);
      end
    end : logging_reg
  end : logging

  // HyperRAM model state machine.
  // Note: this logic is active on _both_ edges of _ckp, so it does not use _ckn.
  //
  // csn is used as an asynchronous reset because the clock is not running when
  // this active low Chip Select signal is deasserted.
  always_ff @(edge ckp, posedge csn, negedge rstn) begin
    if (!rstn || csn) begin
      // The state machine sits in Cmd0 awaiting capture of the first command
      // byte when first it is clocked.
      hram_state <= HRAM_Cmd0;
      hram_drv_oe <= 1'b0;
    end else begin
      case (hram_state)
        HRAM_Cmd0: hram_state <= HRAM_Cmd1;
        HRAM_Cmd1: hram_state <= HRAM_Cmd2;
        HRAM_Cmd2: hram_state <= HRAM_Cmd3;
        HRAM_Cmd3: hram_state <= HRAM_Cmd4;
        HRAM_Cmd4: hram_state <= HRAM_Cmd5;
        HRAM_Cmd5: begin
          case (hram_cmd[2:1])
            2'b11:   hram_state <= HRAM_Ignoring;    // Register reads are ignored.
            2'b01:   hram_state <= HRAM_RegWriting;  // No latency on register writes.
            default: hram_state <= HRAM_Latency1;    // Memory access.
          endcase
        end
        HRAM_Latency1: if (~|hram_lat) hram_state <= hram_cmd[2] ? HRAM_Reading : HRAM_Writing;
        HRAM_Ignoring,
        HRAM_RegWriting,
        HRAM_Reading,
        HRAM_Writing: /* these states persist until CS deasserted */;
        default: hram_state <= HRAM_Cmd0;
      endcase
      case (hram_state)
        HRAM_Cmd0: hram_cmd         <= dq[7:5];
        HRAM_Cmd1: hram_addr[21:19] <= dq[2:0];
        HRAM_Cmd2: hram_addr[18:11] <= dq;
        HRAM_Cmd3: hram_addr[10:3]  <= dq;
        HRAM_Cmd4: /* reserved data */;
        HRAM_Cmd5: begin
          hram_addr[2:0] <= dq[2:0];
          hram_lsb       <= 1'b0;
          // Initialise latency counter; note that we're counting on both clock edges
          // but the programmed latency count is in clock cycles. We also count from [N-1:0],
          // making the mapping from the 'clock latency' in the W956 datasheet to our
          // initial counter value n -> 2(n-1)-1.
          case (latency_initial)
            4'b0000: hram_lat <= 5'h7 << latency_fixed;  // 5 Clock Latency @ 133MHz.
            4'b0001: hram_lat <= 5'h9 << latency_fixed;  // 6 Clock Latency @ 166MHz.
            4'b0010: hram_lat <= 5'hB << latency_fixed;  // 7 Clock Latency @ 200MHz.
            4'b1110: hram_lat <= 5'h3 << latency_fixed;  // 3 Clock Latency @ 83MHz.
            default: hram_lat <= 5'h5 << latency_fixed;  // 4 Clock Latency @ 100MHz.
          endcase
          // TODO: Perhaps make some effort to model the variable latency of a real device?
          // Read state.
          hram_rd_sel    <= 1'b1;
          hram_rd_lsb    <= 1'b0;
        end
        HRAM_Latency1: hram_lat <= hram_lat - 'b1;
        HRAM_RegWriting: begin
          // Retain the upper bytes of the register write data.
          if (!hram_lsb) hram_wdata_hi <= dq;
          hram_lsb <= !hram_lsb;
        end
        HRAM_Writing: begin
          // Retain the MS byte of each 16-bit word received.
          if (!hram_lsb) begin
            hram_wmask_hi <= ~rwds;
            hram_wdata_hi <= dq;
          end
          // Advance the addressing; MS byte of each word is received first.
          hram_lsb  <= !hram_lsb;
          hram_addr <= next_addr;
        end
        HRAM_Reading: begin
          // Advance the addressing; MS byte of each word is returned first.
          hram_lsb  <= !hram_lsb;
          hram_addr <= next_addr;
        end
        default: /* do nothing */;
      endcase
      hram_drv_oe <= hram_re;
      if (hram_drv_oe) begin
        hram_rd_lsb <= !hram_rd_lsb;  // Next byte within 16-bit word.
        hram_rd_sel <= hram_rd_sel ^ hram_rd_lsb;  // Advance after LSB.
      end
    end
  end

  // Read data from HyperRAM model.
  assign dq = hram_drv_oe ? hram_rd_byte : {8{1'bz}};
  // Read strobes from HyperRAM model.
  assign rwds = hram_drv_oe ? !hram_rd_lsb : 1'bz;

  // Some signals have unused bits; use them here to prevent warnings/errors.
  logic unused_hyperram;
  assign unused_hyperram = ^{hram_cfg_wdata, burst_hybrid};

endmodule

