// Copyright lowRISC contributors
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module padring #(
  parameter int unsigned InoutNumber = 1
) (
  input  logic [InoutNumber-1:0] inout_to_pins_i,
  input  logic [InoutNumber-1:0] inout_to_pins_en_i,
  output logic [InoutNumber-1:0] inout_from_pins_o,

  inout  logic [InoutNumber-1:0] inout_pins_io
);
  prim_pad_wrapper_pkg::pad_attr_t pad_attr = '{
    default:'0,
    drive_strength: '1
  };

  prim_pad_wrapper u_inout_pad[InoutNumber] (
    .inout_io (inout_pins_io     ),
    .in_o     (inout_from_pins_o ),
    .ie_i     (1'b1              ),
    .out_i    (inout_to_pins_i   ),
    .oe_i     (inout_to_pins_en_i),
    .attr_i   (pad_attr          ),

    // Don't care
    .in_raw_o   (),
    // Unused in generic and Xilinx variant of the wrapper
    .clk_scan_i (),
    .scanmode_i (),
    .pok_i      ()
  );
endmodule : padring
