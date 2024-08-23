// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module xadc (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // Analog(ue) input width not parametrised due to PCB-specific
  // mapping to XADC pins (see u_xadc below).
  input wire [5:0] analog_p_i,
  input wire [5:0] analog_n_i
);

  // TLUL parameters
  // Specify the 4-byte address width required to cover the whole of the
  // address space assigned to the XADC in the TLUL crossbar.
  // That way, `xadc_adapter` is not susceptible to aliasing.
  localparam int TAW = 10;
  localparam int TDW = 32;

  // DRP 'parameters' (fixed by XADC Hard-IP) to aid readability
  localparam int XAW = 7;
  localparam int XDW = 16;

  logic           drp_dclk;
  logic           drp_den;
  logic           drp_dwe;
  logic [XAW-1:0] drp_daddr;
  logic [XDW-1:0] drp_di;
  logic           drp_drdy;
  logic [XDW-1:0] drp_do;

  // TLUL to DRP adaptor specifically for XADC
  xadc_adapter #(
    .TAw(TAW),
    .TDw(TDW),
    .XAw(XAW),
    .XDw(XDW)
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

  // Respond the cycle after a request
  // TODO: randomise response delay for better bug-finding
  logic respond_d;
  logic respond_q;
  assign respond_d = drp_den;
  always_ff @(posedge drp_dclk or negedge rst_ni) begin
    if (!rst_ni) begin
      respond_q <= '0;
    end else begin
      respond_q <= respond_d;
    end
  end

  // Increment counter when get a request, and use count as response data
  logic [XDW-1:0] req_count_d;
  logic [XDW-1:0] req_count_q;
  assign req_count_d = drp_den ? req_count_q + '1 : req_count_q;
  always @(posedge drp_dclk, negedge rst_ni) begin
    if (!rst_ni) begin
      req_count_q <= '0;
    end else begin
      req_count_q <= req_count_d;
    end
  end

  // Provide signals to the TLUL-DRP adapter
  assign drp_drdy = respond_q;
  assign drp_do = req_count_q;

  // Unused signals from TLUL adapter
  logic unused_drp_dwe, unused_drp_daddr, unused_drp_di;
  assign unused_drp_dwe   =  drp_dwe;
  assign unused_drp_daddr = ^drp_daddr;
  assign unused_drp_di    = ^drp_di;

  // Unused analog(ue) input signals
  logic unused_analog_p, unused_analog_n;
  assign unused_analog_p  = ^analog_p_i;
  assign unused_analog_n  = ^analog_n_i;

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
//  .vauxp0              (0), // Auxiliary channel 0
//  .vauxn0              (0),
//  .vauxp1              (0), // Auxiliary channel 1
//  .vauxn1              (0),
//  .vauxp2              (0), // Auxiliary channel 2
//  .vauxn2              (0),
//  .vauxp3              (0), // Auxiliary channel 3
//  .vauxn3              (0),
    .vauxp4              (analog_p_i[0]), // Auxiliary channel 4
    .vauxn4              (analog_n_i[0]),
    .vauxp5              (analog_p_i[2]), // Auxiliary channel 5
    .vauxn5              (analog_n_i[2]),
    .vauxp6              (analog_p_i[4]), // Auxiliary channel 6
    .vauxn6              (analog_n_i[4]),
//  .vauxp7              (0), // Auxiliary channel 7
//  .vauxn7              (0),
//  .vauxp8              (0), // Auxiliary channel 8
//  .vauxn8              (0),
//  .vauxp9              (0), // Auxiliary channel 9
//  .vauxn9              (0),
//  .vauxp10             (0), // Auxiliary channel 10
//  .vauxn10             (0),
//  .vauxp11             (0), // Auxiliary channel 11
//  .vauxn11             (0),
    .vauxp12             (analog_p_i[1]), // Auxiliary channel 12
    .vauxn12             (analog_n_i[1]),
    .vauxp13             (analog_p_i[3]), // Auxiliary channel 13
    .vauxn13             (analog_n_i[3]),
    .vauxp14             (analog_p_i[5]), // Auxiliary channel 14
    .vauxn14             (analog_n_i[5])
//  .vauxp15             (0), // Auxiliary channel 15
//  .vauxn15             (0)
  );

`endif  // SYNTHESIS

endmodule
