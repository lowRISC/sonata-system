// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This wraps the hbmc_tl_top module, providing sonata-system friendly port
// names and deals with the detailed hyperram parameters.
// It also provides an SRAM model implementation for use in simulations that
// don't want to include the full hyperram controller RTL and BFM (which in
// particular require Xilinx encrypted IP models).
module hyperram import tlul_pkg::*; #(
  parameter HRClkFreq    = 100_000_000,
  parameter HyperRAMSize = 1024 * 1024
) (
  input             clk_i,
  input             rst_ni,

  input             clk_hr_i,
  input             clk_hr90p_i,
  input             clk_hr3x_i,

  input  tl_h2d_t   tl_i,
  output tl_d2h_t   tl_o,

  inout  wire [7:0] hyperram_dq,
  inout  wire       hyperram_rwds,
  output wire       hyperram_ckp,
  output wire       hyperram_ckn,
  output wire       hyperram_nrst,
  output wire       hyperram_cs
);
`ifdef USE_HYPERRAM_SIM_MODEL
  localparam int SRAMModelAddrWidth = $clog2(HyperRAMSize);
  localparam int UnusedParams = HRClkFreq + HyperRAMSize;

  tl_h2d_t unused_tl_b;
  assign unused_tl_b = '0;

  logic [7:0] unused_hyperram_dq;
  logic       unused_hyperram_rwds;
  logic       unused_clk_hr;
  logic       unused_clk_hr90p;
  logic       unused_clk_hr3x;

  assign hyperram_ckp  = '0;
  assign hyperram_ckn  = '0;
  assign hyperram_nrst = '0;
  assign hyperram_cs   = '0;

  assign unused_clk_hr        = clk_hr_i;
  assign unused_clk_hr90p     = clk_hr90p_i;
  assign unused_clk_hr3x      = clk_hr3x_i;

  // TODO: Consider adding extra latency to roughly model the performance of the
  // real hyperram controller
  sram #(
    .AddrWidth       ( SRAMModelAddrWidth ),
    .DataWidth       ( 32                 ),
    .DataBitsPerMask ( 8                  )
  ) u_hyperram_model (
    .clk_i,
    .rst_ni,

    .tl_a_i (tl_i),
    .tl_a_o (tl_o),

    .tl_b_i (unused_tl_b),
    .tl_b_o ()
  );


`else
  hbmc_tl_top #(
    .C_HBMC_CLOCK_HZ(HRClkFreq),
    .C_HBMC_CS_MAX_LOW_TIME_US(4),
    .C_HBMC_FIXED_LATENCY(1'B0),
    .C_IDELAYCTRL_INTEGRATED(1'B0),
    .C_IODELAY_GROUP_ID("HBMC"),
    .C_IODELAY_REFCLK_MHZ(200),
    .C_HBMC_FPGA_DRIVE_STRENGTH(8),
    .C_HBMC_MEM_DRIVE_STRENGTH(46),
    .C_HBMC_FPGA_SLEW_RATE("SLOW"),
    .C_RWDS_USE_IDELAY(1'B0),
    .C_DQ7_USE_IDELAY(1'B0),
    .C_DQ6_USE_IDELAY(1'B0),
    .C_DQ5_USE_IDELAY(1'B0),
    .C_DQ4_USE_IDELAY(1'B0),
    .C_DQ3_USE_IDELAY(1'B0),
    .C_DQ2_USE_IDELAY(1'B0),
    .C_DQ1_USE_IDELAY(1'B0),
    .C_DQ0_USE_IDELAY(1'B0),
    .C_RWDS_IDELAY_TAPS_VALUE(0),
    .C_DQ7_IDELAY_TAPS_VALUE(0),
    .C_DQ6_IDELAY_TAPS_VALUE(0),
    .C_DQ5_IDELAY_TAPS_VALUE(0),
    .C_DQ4_IDELAY_TAPS_VALUE(0),
    .C_DQ3_IDELAY_TAPS_VALUE(0),
    .C_DQ2_IDELAY_TAPS_VALUE(0),
    .C_DQ1_IDELAY_TAPS_VALUE(0),
    .C_DQ0_IDELAY_TAPS_VALUE(0),
    .C_ISERDES_CLOCKING_MODE(0),
    .HyperRAMSize(HyperRAMSize)
  ) u_hbmc_tl_top (
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .clk_hbmc_0(clk_hr_i),
    .clk_hbmc_90(clk_hr90p_i),
    .clk_iserdes(clk_hr3x_i),
    .clk_idelay_ref(1'B0),

    .tl_i(tl_i),
    .tl_o(tl_o),

    .hb_dq(hyperram_dq),
    .hb_rwds(hyperram_rwds),
    .hb_ck_p(hyperram_ckp),
    .hb_ck_n(hyperram_ckn),
    .hb_reset_n(hyperram_nrst),
    .hb_cs_n(hyperram_cs)
  );
`endif
endmodule
