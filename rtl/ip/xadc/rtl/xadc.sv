// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module xadc (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  input analog0_p, // VAUX4P
  input analog0_n, // VAUX4N
  input analog1_p, // VAUX12P
  input analog1_n, // VAUX12N
  input analog2_p, // VAUX5P
  input analog2_n, // VAUX5N
  input analog3_p, // VAUX13P
  input analog3_n, // VAUX13N
  input analog4_p, // VAUX6P
  input analog4_n, // VAUX6N
  input analog5_p, // VAUX14P
  input analog5_n  // VAUX14N
);

  // Specify the 4-byte address width required to cover the whole of the
  // address space assigned to the XADC in the TLUL crossbar.
  // That way, `xadc_adapter` is not susceptible to aliasing.
  localparam int AW = 10;
  localparam int DW = 32;
  localparam int DBW = DW/8;

  logic        drp_dclk;
  logic        drp_den;
  logic        drp_dwe;
  logic  [6:0] drp_daddr;
  logic [15:0] drp_di;
  logic        drp_drdy;
  logic [15:0] drp_do;

  // TLUL to DRP adaptor specifically for XADC
  xadc_adapter #(
    .Aw(AW),
    .Dw(DW)
  ) u_xadc_adapter (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),

    .tl_i        (tl_i),
    .tl_o        (tl_o),

    .drp_dclk_o  (drp_dclk),
    .drp_den_o   (drp_den),
    .drp_dwe_o   (drp_dwe),
    .drp_daddr_o (drp_daddr),
    .drp_di_o    (drp_di),
    .drp_drdy_i  (drp_drdy),
    .drp_do_i    (drp_do)
  );

`ifndef SYNTHESIS
  // Fake XADC - responds with the number of requests submitted

  logic pavlova_drdy_d;
  logic pavlova_drdy_q;
  assign pavlova_drdy_d = drp_den;
  always_ff @(posedge drp_dclk or negedge rst_ni) begin
    if (!rst_ni) begin
      pavlova_drdy_q <= '0;
    end else begin
      pavlova_drdy_q <= pavlova_drdy_d;
    end
  end

  logic [15:0] pavlova_do_d;
  logic [15:0] pavlova_do_q;
  assign pavlova_do_d = drp_den ? pavlova_do_q + '1 : pavlova_do_q;
  always @(posedge drp_dclk, negedge rst_ni) begin
    if (!rst_ni) begin
      pavlova_do_q <= '0;
    end else begin
      pavlova_do_q <= pavlova_do_d;
    end
  end

  assign drp_drdy = pavlova_drdy_q;
  assign drp_do[15:0] = pavlova_do_q;
  assign drp_do[DW-1:16] = '0;

`else  // SYNTHESIS
  // Real XADC
  xadc_wiz_0 u_xadc (
    .reset_in            (0), // Reset signal for the System Monitor control logic

    .dclk_in             (drp_dclk), // Clock input for the dynamic reconfiguration port
    .den_in              (drp_den), // Enable Signal for the dynamic reconfiguration port
    .dwe_in              (drp_dwe), // Write Enable for the dynamic reconfiguration port
    .daddr_in            (drp_daddr), // Address bus for the dynamic reconfiguration port
    .di_in               (drp_di), // Input data bus for the dynamic reconfiguration port
    .drdy_out            (drp_drdy), // Data ready signal for the dynamic reconfiguration port
    .do_out              (drp_do), // Output data bus for dynamic reconfiguration port

    .busy_out            (), // ADC Busy signal
    .channel_out         (), // Channel Selection Outputs
    .eoc_out             (), // End of Conversion Signal
    .eos_out             (), // End of Sequence Signal
    .ot_out              (), // Over-Temperature alarm output
    .vccaux_alarm_out    (), // VCCAUX-sensor alarm output
    .vccint_alarm_out    (), // VCCINT-sensor alarm output
    .user_temp_alarm_out (), // Temperature-sensor alarm output
    .vbram_alarm_out     (),
    .alarm_out           (), // OR'ed output of all the Alarms

    .vp_in               (0), // Dedicated Analog Input Pair
    .vn_in               (0),
    .vauxp0              (0), // Auxiliary channel 0
    .vauxn0              (0),
    .vauxp1              (0), // Auxiliary channel 1
    .vauxn1              (0),
    .vauxp2              (0), // Auxiliary channel 2
    .vauxn2              (0),
    .vauxp3              (0), // Auxiliary channel 3
    .vauxn3              (0),
    .vauxp4              (analog0_p), // Auxiliary channel 4
    .vauxn4              (analog0_n),
    .vauxp5              (analog2_p), // Auxiliary channel 5
    .vauxn5              (analog2_n),
    .vauxp6              (analog4_p), // Auxiliary channel 6
    .vauxn6              (analog4_n),
    .vauxp7              (0), // Auxiliary channel 7
    .vauxn7              (0),
    .vauxp8              (0), // Auxiliary channel 8
    .vauxn8              (0),
    .vauxp9              (0), // Auxiliary channel 9
    .vauxn9              (0),
    .vauxp10             (0), // Auxiliary channel 10
    .vauxn10             (0),
    .vauxp11             (0), // Auxiliary channel 11
    .vauxn11             (0),
    .vauxp12             (analog1_p), // Auxiliary channel 12
    .vauxn12             (analog1_n),
    .vauxp13             (analog3_p), // Auxiliary channel 13
    .vauxn13             (analog3_n),
    .vauxp14             (analog5_p), // Auxiliary channel 14
    .vauxn14             (analog5_n),
    .vauxp15             (0), // Auxiliary channel 15
    .vauxn15             (0)
  );

`endif  // SYNTHESIS

endmodule