// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Derive a Pin-Change Interrupt (PCINT) from a bank of inputs and config.
// There are four modes: rising-edge, falling-edge, either edge, and low level.
// Each input pin is evaluated individually and can each be masked.
// Either synchronised (pre-debounce) or debounced inputs can be used.
// Only one interrupt output is present, which any unmasked input can trigger.
// The interrupt will only stay high while the trigger condition remains true,
// any latching must be done externally.

module pcint #(
  parameter int unsigned Width = 8
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic [1:0]       mode_i,
  input  logic             i_sel_i,
  input  logic [Width-1:0] pin_mask_i,

  input  logic [Width-1:0] sync_i,
  input  logic [Width-1:0] dbnc_i,

  output logic pcint_o
);

  localparam logic[1:0] PCMODE_EDGE = 2'b00; // rising or falling edges
  localparam logic[1:0] PCMODE_RISE = 2'b01; // rising edges only
  localparam logic[1:0] PCMODE_FALL = 2'b10; // falling edges only
  localparam logic[1:0] PCMODE_LOW  = 2'b11; // low-level (not edge based)

  logic [Width-1:0] prev_in_q;
  logic pcint_q;

  logic [Width-1:0] curr_in;
  logic [Width-1:0] rising;
  logic [Width-1:0] falling;
  logic [Width-1:0] pn_int;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      prev_in_q <= '0;
      pcint_q <= '0;
    end else begin
      prev_in_q <= curr_in;
      pcint_q <= (|pn_int); // combined masked pin-specific interrupts
    end
  end

  assign curr_in = i_sel_i ? dbnc_i : sync_i;

  assign rising  =   curr_in  & (~prev_in_q);
  assign falling = (~curr_in) &   prev_in_q;

  for (genvar pn = 0; pn < Width; pn++) begin
    assign pn_int[pn] = pin_mask_i[pn] && (
      (mode_i == PCMODE_LOW && !curr_in[pn]) ||
      ((mode_i == PCMODE_RISE || mode_i == PCMODE_EDGE) && rising[pn]) ||
      ((mode_i == PCMODE_FALL || mode_i == PCMODE_EDGE) && falling[pn])
    );
  end

  assign pcint_o = pcint_q;

endmodule
