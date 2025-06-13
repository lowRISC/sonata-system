// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/*
 * spi_core - SPI controller driver, bus interface and transmit/receive FIFOs are implemented
 * outside this module.
 *
 * The SPI clock will pause if a new byte is needed but none is available on the data_in interface
 * or if a byte is ready on the data_out interface and it cannot be accepted. In particular this
 * means an SPI transaction can be started without needing to guarantee all bytes to transmit will
 * be immediately available and all bytes to receive can be immediately consumed.
 *
 * data_in_X - Interface for data being transmitted. data_in_ready_o indicates spi_core wants a new
 * byte to transmit, data_in_valid_i indicates data_in_i is valid. When both are asserted the byte
 * is accepted and the following cycle either data_in_valid_i must drop or the next byte must be
 * available on data_in_i.
 *
 * data_out_X - Interface for data received. data_out_valid_o indicates spi_core has received
 * a full byte which is available on data_out_o. data_out_ready_i indicates the byte can be
 * accepted. When both are asserted the byte is consumed and data_out_valid_o will drop the
 * following cycle (as spi_core begins receiving the next byte if any remain).
 *
 * start_i/byte_count_i/idle_o - Command and status interface. When idle_o is asserted a start can
 * be issued. Asserting start_i will cause spi_core to transmit/receive byte_coubt_i bytes. When
 * spi_core is active (idle_o is low) start_i is ignored.
 *
 * cpol_i/cpha_i/msb_first_i/half_clk_period_i - SPI configuration. Must not change whilst spi_core
 * is active (idle_o is low)
 *  * cpol_i - clock polarity (0 == idle clock is low)
 *  * cpha_i - clock phase (0 == data sampled on positive clock edge)
 *  * msb_first_i - 1 == first bit of a byte is the most significant bit
 *  * half_clock_period_i - Length of half of an SPI clock period measured in cycles of clk_i.
 *    value | clock speed (relative to clk_i)
 *    ----- | -------------------------------
 *      0   | 1/2 (1 clock per half period)
 *      1   | 1/4 (2 clocks per half period)
 *      2   | 1/6 (3 clocks per half period)
 *
 * spi_copi_o/spi_cipo_i/spi_clk_o - SPI interface IO
 */

module spi_core #(
  CLK_COUNT_W = 16,
  BYTE_COUNT_W = 11
) (
  input clk_i,
  input rst_ni,

  input sw_reset_i,

  input  logic [7:0] data_in_i,
  input  logic       data_in_valid_i,
  output logic       data_in_ready_o,

  output logic [7:0] data_out_o,
  output logic       data_out_valid_o,
  input  logic       data_out_ready_i,

  input  logic                    start_i,
  input  logic [BYTE_COUNT_W-1:0] byte_count_i,
  output logic                    idle_o,

  input  logic cpol_i,
  input  logic cpha_i,
  input  logic msb_first_i,
  input  logic [CLK_COUNT_W-1:0] half_clk_period_i,
  input  logic copi_idle_i,

  output logic spi_copi_o,
  input  logic spi_cipo_i,
  output logic spi_clk_o
);
  // All logic is clocked from clk_i (as opposed to clocking some from the generated SPI clock).
  // clk_counter_q is used to generate the spi clock. When clk_counter_q reaches half_clk_period_i
  // the clock inverts.
  logic [CLK_COUNT_W-1:0] clk_counter_q, clk_counter_d;

  typedef enum logic [1:0] {
    IDLE,
    WAIT_START,
    ACTIVE
  } state_e;

  state_e state_q, state_d;

  // The clock is separated into two phases the sample phase and the output phase. Data is sampled
  // at the clock edge at the beginning of the sample phase and data is output at the clock edge at
  // the beginning of the output phase. The `output_edge` and `sample_edge` signals below are
  // asserted just before the relevant edge.
  //
  // In the diagram below 'spi_dat' represents a single bit on the SPI bus (in both directions). It
  // uses the phase cpha_i == 1 with clock polarity cpol_i == 0. This means data is output on the
  // positive edge and sampled on the negative edge.
  //
  //                 ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐
  // clk_i         : ┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──
  //                       ┌───────────┐           ┌───────────┐           ┌───────────┐
  // spi_clk_o     : ──────┘           └───────────┘           └───────────┘           └───────────
  //                 ──────┐           ┌───────────┐           ┌───────────┐           ┌───────────
  // sample_phase_q:       └───────────┘           └───────────┘           └───────────┘
  //                 xxxxxx╱                      ╲╱                      ╲╱                      ╲
  // spi_dat       : xxxxxx╲          0           ╱╲          1           ╱╲          2           ╱
  //                             ┌─────┐                 ┌─────┐                 ┌─────┐
  // sample_edge_q : ────────────┘     └─────────────────┘     └─────────────────┘     └───────────
  //                 ┌─────┐                 ┌─────┐                 ┌─────┐
  // output_edge_q : ┘     └─────────────────┘     └─────────────────┘     └───────────────────────
  //
  // start_edge is set to output the first bit of a byte in a transaction for cpha_i == 0 when the
  // first bit is output before a clock edge.
  // finish_edge is set for the final edge of the transaction.

  logic active;
  logic output_edge, sample_edge, start_edge, finish_edge;

  logic clk_q, clk_d;
  logic sample_phase_q, sample_phase_d;

  logic [BYTE_COUNT_W-1:0] byte_count_q, byte_count_d;
  logic [3:0] bit_count_q, bit_count_d;

  logic [7:0] copi_shift_q, copi_shift_d;
  logic [7:0] cipo_shift_q, cipo_shift_d;

  logic data_out_valid_q, data_out_valid_d;

  logic clk_running;

  assign active = state_q == ACTIVE;

  function automatic logic [7:0] bit_reverse(input logic [7:0] in);
    logic [7:0] out;
    for (int unsigned b = 0; b < 8; b++) begin
      out[b] = in[7 - b];
    end
    return out;
  endfunction

  always_comb begin
    clk_running = 1'b0;
    finish_edge  = 1'b0;

    if (active) begin
      if (bit_count_q != 0) begin
        clk_running = 1'b1;
      end else if (clk_counter_q < half_clk_period_i) begin
        clk_running = 1'b1;
      end else if (sample_phase_q) begin
        if (byte_count_q > 1) begin
          clk_running = data_in_valid_i;
        end else begin
          clk_running = 1'b0;
          finish_edge = 1'b1;
        end
      end else begin
        clk_running = data_out_ready_i;
      end
    end
  end

  always_comb begin
    clk_d          = clk_q;
    clk_counter_d  = clk_counter_q;
    sample_phase_d = sample_phase_q;
    sample_edge    = 1'b0;
    output_edge    = 1'b0;

    if (clk_running) begin
      if (clk_counter_q == half_clk_period_i) begin
        clk_counter_d = '0;

        if (sample_phase_q) begin
          output_edge = ~finish_edge;
        end else begin
          sample_edge = 1'b1;
        end

        clk_d          = ~clk_q;
        sample_phase_d = ~sample_phase_q;
      end else begin
        clk_counter_d = clk_counter_q + 1'b1;
      end
    end else if (~active || finish_edge) begin
      clk_d          = cpol_i;
      sample_phase_d = cpha_i;
      clk_counter_d  = '0;
    end
  end

  always_ff @(posedge clk_i) begin
    clk_q          <= clk_d;
    clk_counter_q  <= clk_counter_d;
    sample_phase_q <= sample_phase_d;
  end

  always_comb begin
    copi_shift_d    = copi_shift_q;
    data_in_ready_o = 1'b0;

    if (output_edge || start_edge) begin
      if ((bit_count_q == 4'd0) || (bit_count_q == 4'd8) || start_edge) begin
        copi_shift_d = msb_first_i ? data_in_i : bit_reverse(data_in_i);
        data_in_ready_o = 1'b1;
      end else begin
        copi_shift_d = {copi_shift_q[6:0], copi_idle_i};
      end
    end else if (finish_edge) begin
      // Return the COPI line to its idle state.
      copi_shift_d = {copi_shift_q[6:0], copi_idle_i};
    end
  end

  always_ff @(posedge clk_i) begin
    copi_shift_q <= copi_shift_d;
  end

  assign data_out_valid_d = sample_edge && (bit_count_q == '0);
  assign cipo_shift_d = {cipo_shift_q[6:0], spi_cipo_i};

  always_ff @(posedge clk_i) begin
    if (sample_edge) begin
      cipo_shift_q <= cipo_shift_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      data_out_valid_q <= 1'b0;
    end else begin
      data_out_valid_q <= data_out_valid_d;
    end
  end

  always_comb begin
    state_d = state_q;
    bit_count_d = bit_count_q;
    byte_count_d = byte_count_q;
    start_edge = 1'b0;

    case (state_q)
      IDLE: begin
        if (start_i) begin
          if (data_in_valid_i) begin
            state_d = ACTIVE;
            start_edge = !cpha_i;
          end else begin
            state_d = WAIT_START;
          end

          bit_count_d  = cpha_i ? 4'd8 : 4'd7;
          byte_count_d = byte_count_i;
        end
      end
      WAIT_START: begin
        if (data_in_valid_i) begin
          state_d = ACTIVE;
          start_edge = !cpha_i;
        end
      end
      ACTIVE: begin
        if (output_edge || finish_edge) begin
          if (bit_count_q == 4'd0) begin
            if (byte_count_q == 1) begin
              state_d = IDLE;
            end else begin
              byte_count_d = byte_count_q - 1'b1;
              bit_count_d = 4'd7;
            end
          end else begin
            bit_count_d = bit_count_q - 1'b1;
          end
        end
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      state_q <= IDLE;
    end else if (sw_reset_i) begin
      state_q <= IDLE;
    end else begin
      state_q <= state_d;
    end
  end

  always_ff @(posedge clk_i) begin
    bit_count_q  <= bit_count_d;
    byte_count_q <= byte_count_d;
  end

  assign spi_clk_o  = clk_q;
  assign spi_copi_o = copi_shift_q[7];

  assign data_out_o = msb_first_i ? cipo_shift_q : bit_reverse(cipo_shift_q);
  assign data_out_valid_o = data_out_valid_q;

  assign idle_o = state_q == IDLE;
endmodule
