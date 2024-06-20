// Copyright lowRISC contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * This module will assert it's output, `modulated_o`,
 * when `impulse_i` is asserted. When `impulse_i` is dropped,
 * it will fade it's output by decreasing it's pulse width.
 *
 * Total fade time = (1 << PwmCounterSize)^2 * NumPeriods * the clock period
 * The default fade time is 2.24 second, when the clock is 30MHz
 */
module pwm_fade #(
  /// The size of the pwm counter.
  parameter PwmCounterSize = 7,
  /// The number of pwm periods between pulse width reductions.
  parameter NumPeriods = (1 << 12) - 1
)(
  input logic clk_i,
  input logic rst_ni,
  input logic impulse_i,

  output logic modulated_o
);
  localparam PwmCounterMax = (1 << PwmCounterSize) - 1;

  logic [$clog2(NumPeriods+1)+PwmCounterSize:0] counter;
  logic [PwmCounterSize-1:0] pwm_counter, pulse_width;

  // The bottom `PwmCounterSize` bits of the counter
  // are used as the pwm counter.
  assign pwm_counter = counter[PwmCounterSize-1:0];

  assign modulated_o = pwm_counter < pulse_width;

  always_ff @(posedge clk_i or negedge rst_ni) begin : main
    if (!rst_ni) begin
      pulse_width <= 0;
      counter <= 0;
    end else if (impulse_i != 0) begin
      pulse_width <= PwmCounterMax;
      counter <= 0;
    end else if (pulse_width != 0) begin
      if (counter == (NumPeriods << PwmCounterSize)) begin
        pulse_width <= pulse_width - 1;
        counter <= 0;
      end else begin
        counter <= counter + 1;
      end
    end
  end : main
endmodule : pwm_fade
