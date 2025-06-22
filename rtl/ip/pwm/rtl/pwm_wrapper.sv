// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This wrapper instantiates a series of PWMs and distributes requests from the device bus.
module pwm_wrapper #(
  parameter int unsigned PwmWidth   = 12,
  parameter int unsigned PwmCtrSize = 8,
  parameter int unsigned DataWidth  = 32,
  parameter int unsigned RegAddrWidth = 12
) (
  input  logic clk_i,
  input  logic rst_ni,

  // TileLink interface to registers.
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Collected output of all PWMs.
  output logic [PwmWidth-1:0] pwm_o
);

  localparam int unsigned PwmIdxOffset = $clog2(DataWidth / 8) + 1;
  localparam int unsigned PwmIdxWidth = RegAddrWidth - PwmIdxOffset;

  // IO for device bus.
  logic [RegAddrWidth-1:0] device_addr;
  logic                    device_we;
  logic [DataWidth/8-1:0]  device_be;
  logic [DataWidth-1:0]    device_wdata;
  logic [DataWidth-1:0]    device_rdata;

  tlul_adapter_reg #(
    .AccessLatency ( 0            ),
    .RegAw         ( RegAddrWidth )
  ) pwm_device_adapter (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    // TL-UL interface.
    .tl_i         (tl_i),
    .tl_o         (tl_o),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (),  // Not readable.
    .we_o         (device_we),
    .addr_o       (device_addr),
    .wdata_o      (device_wdata),
    .be_o         (device_be),
    .busy_i       ('0),
    .rdata_i      (device_rdata),
    .error_i      ('0)
  );

  // Generate PwmWidth number of PWMs.
  for (genvar i = 0; i < PwmWidth; i++) begin : gen_pwm
    logic [PwmCtrSize-1:0] data_d;
    logic [PwmCtrSize-1:0] counter_q;
    logic [PwmCtrSize-1:0] pulse_width_q;
    logic counter_en;
    logic pulse_width_en;
    logic [PwmIdxWidth-1:0] pwm_idx;

    assign pwm_idx = i;

    // Byte enables are currently unsupported for PWM.
    assign data_d         = device_wdata[PwmCtrSize-1:0]; // Only take PwmCtrSize LSBs.
    // Each PWM has a 64-bit block. The most significant 32 bits are the counter and the least
    // significant 32 bits are the pulse width.
    assign counter_en     = device_we & (device_addr[RegAddrWidth-1:PwmIdxOffset] == pwm_idx)
                                      & device_addr[PwmIdxOffset-1];
    assign pulse_width_en = device_we & (device_addr[RegAddrWidth-1:PwmIdxOffset] == pwm_idx)
                                      & ~device_addr[PwmIdxOffset-1];

    always @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        counter_q       <= '0;
        pulse_width_q   <= '0;
      end else begin
        if (counter_en) begin
          counter_q     <= data_d;
        end
        if (pulse_width_en) begin
          pulse_width_q <= data_d;
        end
      end
    end
    pwm #(
      .CtrSize( PwmCtrSize )
    ) u_pwm (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .pulse_width_i(pulse_width_q),
      .max_counter_i(counter_q),
      .modulated_o  (pwm_o[i])
    );
  end : gen_pwm

  // Generating the device bus output.
  // TODO: Reading from PWM currently not possible.
  assign device_rdata = '0;

  logic _unused;
  assign _unused = ^{device_be, device_wdata};
endmodule
