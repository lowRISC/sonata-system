// Copyright lowRISC contributors
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// rtl/system/pinmux.sv is automatically generated using util/top_gen.py from rtl/templates/pinmux.sv.tpl
// Please make any edits to the template file.

module pinmux
  import sonata_pkg::*;
(
  // Clock and reset.
  input logic clk_i,
  input logic rst_ni,

  // List of block IOs.
  % for label, width, name, instances in block_ios:
  ${label} logic ${width}${name}[${instances}],
  % endfor

  // Pin Signals
  input  sonata_in_pins_t  in_from_pins_i,
  output sonata_out_pins_t out_to_pins_o,
  output sonata_out_pins_t out_to_pins_en_o,

  input  sonata_inout_pins_t inout_from_pins_i,
  output sonata_inout_pins_t inout_to_pins_o,
  output sonata_inout_pins_t inout_to_pins_en_o,

  // TileLink interfaces.
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);
  // Local parameters.
  localparam int unsigned RegAddrWidth = 12;
  localparam int unsigned BusDataWidth = 32;

  // Register control signals.
  logic reg_we;
  logic [RegAddrWidth-1:0] reg_addr;
  /* verilator lint_off UNUSEDSIGNAL */
  logic [BusDataWidth-1:0] reg_wdata;
  /* verilator lint_on UNUSEDSIGNAL */
  logic [(BusDataWidth/8)-1:0] reg_be;
  logic [BusDataWidth-1:0] reg_rdata;

  logic unused_reg_signals;

  //TODO allow reading selector values.
  assign reg_rdata = BusDataWidth'('0);

  tlul_adapter_reg #(
    .RegAw            ( RegAddrWidth ),
    .RegDw            ( BusDataWidth ),
    .AccessLatency    ( 1            )
  ) u_tlul_adapter_reg (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),

    // TL-UL interface.
    .tl_i         (tl_i),
    .tl_o         (tl_o),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (),
    .we_o         (reg_we),
    .addr_o       (reg_addr),
    .wdata_o      (reg_wdata),
    .be_o         (reg_be),
    .busy_i       (1'b0),
    .rdata_i      (reg_rdata),
    .error_i      (1'b0)
  );

  // Outputs - Blocks IO is muxed to choose which drives the output and output
  // enable of a physical pin
  % for output_idx, (pin, possible_block_outputs, num_options) in enumerate(output_pins):

  logic [${num_options - 1}:0] ${pin.name}_sel;
  logic ${pin.name}_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ${pin.name}_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == ${output_idx - output_idx%4} &
    reg_be[${output_idx%4}] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ${pin.name}_sel <= ${num_options}'b10;
    end else begin
      if (reg_we & ${pin.name}_sel_addressed) begin
        ${pin.name}_sel <= reg_wdata[${(output_idx%4)*8}+:${num_options}];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${num_options})
  ) ${pin.name}_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      % for idx, bio in enumerate(possible_block_outputs):
      ${bio.uid.block}_${bio.uid.io}_i[${bio.uid.instance}]${bio.io_idx_str}${',' if idx < (num_options - 2) else ''}
      % endfor
    }),
    .sel_i(${pin.name}_sel),
    .out_o(${pin.direction_prefix}to_pins_o[${pin.idx_param}])
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${num_options})
  ) ${pin.name}_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      % for idx, bio in enumerate(possible_block_outputs):
      ${f"{bio.uid.block}_{bio.uid.io}_en_i[{bio.uid.instance}]{bio.io_idx_str}" if bio.is_inout else "1'b1"}${',' if idx < (num_options - 2) else ''}
      % endfor
    }),
    .sel_i(${pin.name}_sel),
    .out_o(${pin.direction_prefix}to_pins_en_o[${pin.idx_param}])
  );
  % endfor

  // Inputs - Physical pin inputs are muxed to particular block IO
  % for input_idx, (block_io, possible_pins, num_options) in enumerate(output_block_ios):

  logic [${num_options-1}:0] ${block_io.name}_sel;
  logic ${block_io.name}_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign ${block_io.name}_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == ${input_idx - input_idx%4} &
    reg_be[${input_idx%4}] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ${block_io.name}_sel <= ${num_options}'b10;
    end else begin
      if (reg_we & ${block_io.name}_sel_addressed) begin
        ${block_io.name}_sel <= reg_wdata[${(input_idx%4)*8}+:${num_options}];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${num_options})
  ) ${block_io.name}_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b${block_io.default_value},
      % for idx, pin in enumerate(possible_pins):
      ${pin.direction_prefix}from_pins_i[${pin.idx_param}]${',' if idx < len(possible_pins)-1 else ''}
      % endfor
      % if len(possible_pins) == 0:
      1'b${block_io.default_value}
      % endif
    }),
    .sel_i(${block_io.name}_sel),
    .out_o(${block_io.uid.block}_${block_io.uid.io}_o[${block_io.uid.instance}]${block_io.io_idx_str})
  );
  % endfor

  // Combining inputs for combinable inouts
  % for block_io, default_value, operator, pins_and_select_values in combined_input_block_ios:
  assign ${block_io.uid.block}_${block_io.uid.io}_o[${block_io.uid.instance}] =
    % for idx, (pin, select_value) in enumerate(pins_and_select_values):
    (${pin.name}_sel == ${select_value} ? ${pin.direction_prefix}from_pins_i[${pin.idx_param}] : ${default_value})${operator if idx < len(pins_and_select_values) - 1 else ';'}
    % endfor
  % endfor
endmodule
