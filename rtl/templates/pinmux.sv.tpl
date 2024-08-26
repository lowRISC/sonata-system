// Copyright lowRISC contributors
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// rtl/system/pinmux.sv is automatically generated using util/top_gen.py from rtl/templates/pinmux.sv.tpl
// Please make any edits to the template file.

module pinmux (
  // Clock and reset
  input logic clk_i,
  input logic rst_ni,

  // Blocks
  % for label, width, name, instances in block_ios:
  ${label} logic ${width}${name}[${instances}],
  % endfor

  // Pins
  % for width, name in pin_ios:
  inout logic ${width}${name},
  % endfor

  // TileLink interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);

  //TODO allow selectors to be driven by TileLink interface for outputs and inputs.
  // TODO connect up TileLink interfaces
  assign tl_o = '0;

  // Outputs - Blocks IO is muxed to choose which drives the output and output
  // enable of a physical pin
  % for pin_output, idx_str, idx_alt, possible_blocks in output_list:

  logic [${len(possible_blocks)}:0] ${pin_output}${idx_str}_sel;
  logic ${pin_output}${idx_str}_o;
  logic ${pin_output}${idx_str}_en_o;

  assign ${pin_output}${idx_str}_sel = 'b10;

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${len(possible_blocks)+1})
  ) ${pin_output}${idx_str}_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'bz,
      % for idx, (block, io, inst, bit_str, _) in enumerate(possible_blocks):
      ${block}_${io}_i[${inst}]${bit_str}${',' if idx < len(possible_blocks)-1 else ''}
      % endfor
    }),
    .sel_i(${pin_output}${idx_str}_sel),
    .out_o(${pin_output}${idx_str}_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${len(possible_blocks)+1})
  ) ${pin_output}${idx_str}_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      % for idx, (block, io, inst, bit_str, is_inout) in enumerate(possible_blocks):
      ${block + '_' + io + '_en_i[' + str(inst) + ']' + bit_str if is_inout else "'1"}${',' if idx < len(possible_blocks)-1 else ''}
      % endfor
    }),
    .sel_i(${pin_output}${idx_str}_sel),
    .out_o(${pin_output}${idx_str}_en_o)
  );

  assign ${pin_output}${idx_alt} = ${pin_output}${idx_str}_en_o ? ${pin_output}${idx_str}_o : 1'bz;
  % endfor

  // Inputs - Physical pin inputs are muxed to particular block IO
  % for block_input, inst, bit_idx, bit_str, possible_pins in input_list:

  logic [${len(possible_pins)-1}:0] ${block_input}_${inst}${bit_str}_sel;

  assign ${block_input}_${inst}${bit_str}_sel = 'b10;

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${len(possible_pins)})
  ) ${block_input}_${inst}${bit_str}_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      % for idx, pin in enumerate(possible_pins):
      ${pin}${',' if idx < len(possible_pins)-1 else ''}
      % endfor
    }),
    .sel_i(${block_input}_${inst}${bit_str}_sel),
    .out_o(${block_input}_o[${inst}]${'' if bit_str == '' else '['+str(bit_idx)+']'})
  );
  % endfor

  // Combining inputs for combinable inouts
  % for block_input, inst, combine_pins, combine_type in combine_list:
  assign ${block_input}_o[${inst}] =
    % for idx, pin in enumerate(combine_pins):
    ${pin}${';' if idx == len(combine_pins)-1 else ' &&' if combine_type == 'and' else ' ||'}
    % endfor
  % endfor
endmodule
