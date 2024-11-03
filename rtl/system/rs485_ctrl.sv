// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Controls the RS-485 transceiver. RS-485 is half-duplex so cannot transmit and receive
// simultaneously. This module handles enabling and disabling the driver and receiver as appropriate
// and gating the incoming receive line to prevent glitches as the receiver is enabled and disabled
// and prevent transmitted bits looping back.
//
// When enabling the driver there needs to be a short pause before transmission begins to give time
// for the driver to enable. Similarly when disabling the driver and switching back to receive there
// needs to be a short pause for the receiver to enable. For simplicitly the same time is used for
// both and is given by 'TransceiverSwitchCycles'.
//
// Additionally when tranmission ends there needs to be a delay to allow the idle (high) line state
// to be driven by the RS-485 driver before the driver is disabled given by `TransmitEndCycles`. The
// Sonata RS-485 transceiver's receiver should output 1 (the idle high state) when the RS-485 lines
// are undriven (as documented in the datasheet) but it has been observed that this only works if
// the idle state is driven by the driver before it is disabled.
//
// As the underlying UART used has no way to delay transmission for the driver to enable this module
// delays the transmision with a shift register so the driver can be enabled a suitable amount of
// time before the beginning of the start bit.
//
// The `tx_i`/`rx_o` connect to the relevant UART IO lines. `tx_enable` should be asserted any time
// the UART is transmitting. `rx_enable` should be asserted when the RS-485 receiver should be
// enabled. It should remain asserted whilst transmission is occurring and `rx_enable` should be
// asserted at least `TransceiverSwitchCycles` ahead of the UART listening on `rx_o` this is to
// prevent the UART seeing false transitions on `rx_o`.
module rs485_ctrl #(
  parameter int unsigned TransceiverSwitchCycles = 4,
  parameter int unsigned TransmitEndCycles       = 4
) (
  input clk_i,
  input rst_ni,

  input  logic tx_i,
  output logic rx_o,
  input  logic rx_enable_i,
  input  logic tx_enable_i,

  output logic di_o,
  output logic ren_o,
  output logic de_o,
  input  logic ro_i
);
  // Time to wait in cycles before disabling the driver after transmission finished. Needs to factor
  // in TransceiverSwitchCycles as the transmission is delayed by that many cycles through the
  // `di_delay` shift register.
  localparam int unsigned TransmitCooldownCount = TransceiverSwitchCycles + TransmitEndCycles;
  localparam int unsigned CounterW = $clog2(TransmitCooldownCount+1);

  logic [TransceiverSwitchCycles-1:0] di_delay;
  logic                               ren_q, ren_d;
  logic                               de_q, de_d;
  logic                               rx_q, rx_d;
  logic                               rx_gate;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      di_delay <= '1;
      ren_q    <= 1'b1;
      de_q     <= 1'b0;
      rx_q     <= 1'b1;
    end else begin
      di_delay <= {di_delay[TransceiverSwitchCycles-2:0], tx_i};
      ren_q    <= ren_d;
      de_q     <= de_d;
      rx_q     <= rx_d;
    end
  end

  // When `rx_gate` is set force the output RX to 1 (idle state). Output RX is flopped to ease
  // timing and avoid any glitches.
  assign rx_d = rx_gate ? 1'b1 : ro_i;

  // The module starts in `ReceiveIdle` state where the RS-485 receiver is enabled with any incoming
  // communications passed through (provided `rx_enable_i` is asserted). When tranmission starts
  // (`tx_enable_i`) is asserted it moves immediately to `Transmit` where the driver is enabled and
  // the receiver disabled. When the tranmission has finished (`tx_enable_i` is deasserted) it
  // transitions through `TransmitCooldown` and `ReceiverStartWait` with a short delay in each.
  // These states handle disabling the driver/enabling the receiver and un-gating the RX line before
  // returning to the `ReceiveIdle` state.
  typedef enum logic [1:0] {
    ReceiveIdle,
    Transmit,
    TransmitCooldown,
    ReceiveStartWait
  } state_e;

  state_e              state_q, state_d;
  logic [CounterW-1:0] counter_q, counter_d;

  always_comb begin
    state_d   = state_q;
    ren_d     = ren_q;
    de_d      = de_q;
    counter_d = counter_q;
    rx_gate   = 1'b0;

    case (state_q)
      ReceiveIdle: begin
        ren_d   = ~rx_enable_i;
        de_d    = 1'b0;
        rx_gate = 1'b0;
      end
      Transmit: begin
        // Transmitting, enable the driver, disable the receiver and gate the RX
        ren_d   = 1'b1;
        de_d    = 1'b1;
        rx_gate = 1'b1;

        if (~tx_enable_i) begin
          // Trasnmission has finished, move to cooldown
          counter_d = CounterW'(TransmitCooldownCount);
          state_d   = TransmitCooldown;
        end
      end
      TransmitCooldown: begin
        // Transmission has just finished, waiting for a delay before disabling the driver.
        // Always gate the RX in this state.
        rx_gate = 1'b1;

        if (counter_q != 0) begin
          // Whilst still waiting disable driver and receiver and continue counting down
          ren_d     = 1'b1;
          de_d      = 1'b1;
          counter_d = counter_q - 1'b1;
        end else begin
          // Delay is complete, disable the driver and enable the receiver (if `rx_enable_i` is
          // asserted). Move to `ReceiveStateWait`
          ren_d     = ~rx_enable_i;
          de_d      = 1'b0;
          state_d   = ReceiveStartWait;
          counter_d = CounterW'(TransceiverSwitchCycles);
        end
      end
      ReceiveStartWait: begin
        // Transmission has finished, driver just been disable and receiver enabled. Wait a short
        // period with RX gated for receiver to fully enable to prevent any glitches reaching the
        // UART
        ren_d = ~rx_enable_i;
        de_d  = 1'b0;

        if (counter_q != 0) begin
          // Whilst still waiting keep RX gated
          rx_gate   = 1'b1;
          counter_d = counter_q - 1'b1;
        end else begin
          // Delay is complete, ungate RX and move back to `ReceiveIdle`
          rx_gate = 1'b0;
          state_d = ReceiveIdle;
        end
      end
    endcase

    if (state_q != Transmit) begin
      // If we're not in transmit and we see `tx_enable_i` always go immediately to `Transmit` along
      // with enabling the driver, disabling the receiver and gating the RX line. This ensures we
      // correctly handle new transmissions that start whilst we're still running through
      // `TransmitCooldown` and `ReceiveStartWait` from a previous tranmission.
      if (tx_enable_i) begin
        ren_d   = 1'b1;
        de_d    = 1'b1;
        rx_gate = 1'b1;
        state_d = Transmit;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= ReceiveIdle;
    end else begin
      state_q <= state_d;
    end
  end

  always_ff @(posedge clk_i) begin
    counter_q <= counter_d;
  end

  assign rx_o  = rx_q;
  assign di_o  = di_delay[TransceiverSwitchCycles-1];
  assign ren_o = ren_q;
  assign de_o  = de_q;
endmodule
