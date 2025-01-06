// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Registers for controlling the hardware revoker in CHERIoT Ibex.

module rev_ctl import rev_ctl_reg_pkg::*; (
  // Clock and reset.
  input logic clk_i,
  input logic rst_ni,

  // Interface with core.
  input  logic [ 63:0] core_to_ctl_i,
  output logic [127:0] ctl_to_core_o,
  output logic         rev_ctl_irq_o,

  // TileLink interfaces.
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);
  rev_ctl_reg2hw_t reg2hw;
  rev_ctl_hw2reg_t hw2reg;

  // Internal registers.
  logic go;

  // Internal wires.
  logic done;
  assign done = reg2hw.epoch.running.q & ~core_to_ctl_i[0];

  // Architectural registers.
  logic interrupt_status;

  always @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      // Clear interrupt on reset.
      interrupt_status <= 1'b0;
    end else begin
      if (reg2hw.interrupt_status.qe) begin
        // Any value written to the status clears it.
        interrupt_status <= 1'b0;
      end else if (reg2hw.interrupt_enable.q) begin
        // When interrupts are enabled, automatically clear the status because
        // the PLIC will communicate this to the core.
        interrupt_status <= 1'b0;
      end else if (done) begin
        // When done raise an interrupt.
        interrupt_status <= 1'b1;
      end
    end
  end

  // Control to core.
  always @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      go <= 1'b0;
    end else begin
      if (reg2hw.go.qe) begin
        go <= 1'b1;
      end else begin
        go <= 1'b0;
      end
    end
  end

  assign ctl_to_core_o = {63'h0, go, reg2hw.end_addr.q, reg2hw.start_addr.q};
  assign rev_ctl_irq_o = done & reg2hw.interrupt_enable.q;

  // Hardware to registers.
  // Read values for hardware extended registers.
  assign hw2reg.go.d = 32'h5500_0000;
  assign hw2reg.interrupt_status.d = interrupt_status;
  // Increment epoch when done with a sweep.
  assign hw2reg.epoch.epoch.d = reg2hw.epoch.epoch.q + 31'b1;
  assign hw2reg.epoch.epoch.de = done;
  // Least significant bit from the core indicates whether the revoker is
  // currently running or not.
  assign hw2reg.epoch.running.d = core_to_ctl_i[0];
  assign hw2reg.epoch.running.de = 1'b1;

  // Instantiate the registers.
  rev_ctl_reg_top u_rev_ctl_reg_top (
    .clk_i,
    .rst_ni,
    .reg2hw,
    .hw2reg,
    .tl_i,
    .tl_o
  );

  logic _unused_input;
  assign _unused_input = |core_to_ctl_i[63:1];
endmodule
