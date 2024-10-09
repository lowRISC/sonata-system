// Copyright lowRISC contributors
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// rtl/system/pinmux.sv is automatically generated using util/top_gen.py from rtl/templates/pinmux.sv.tpl
// Please make any edits to the template file.

module pinmux (
  // Clock and reset.
  input logic clk_i,
  input logic rst_ni,

  // List of block IOs.
  % for label, width, name, instances in block_ios:
  ${label} logic ${width}${name}[${instances}],
  % endfor

  // List of pins.
  % for width, name in pin_ios:
  inout logic ${width}${name},
  % endfor

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
  % for output_idx, (pin_output, idx_str, idx_alt, possible_blocks) in enumerate(output_list):

  logic [${len(possible_blocks)}:0] ${pin_output}${idx_str}_sel;
  logic ${pin_output}${idx_str}_o;
  logic ${pin_output}${idx_str}_en_o;
  logic ${pin_output}${idx_str}_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ${pin_output}${idx_str}_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == ${output_idx - output_idx%4} &
    reg_be[${output_idx%4}] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ${pin_output}${idx_str}_sel <= ${len(possible_blocks)+1}'b10;
    end else begin
      if (reg_we & ${pin_output}${idx_str}_sel_addressed) begin
        ${pin_output}${idx_str}_sel <= reg_wdata[${(output_idx%4)*8}+:${len(possible_blocks)+1}];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(${len(possible_blocks)+1})
  ) ${pin_output}${idx_str}_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
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
  % for input_idx, (block_input, inst, bit_idx, bit_str, possible_pins) in enumerate(input_list):

  logic [${len(possible_pins)-1}:0] ${block_input}_${inst}${bit_str}_sel;
  logic ${block_input}_${inst}${bit_str}_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign ${block_input}_${inst}${bit_str}_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == ${input_idx - input_idx%4} &
    reg_be[${input_idx%4}] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ${block_input}_${inst}${bit_str}_sel <= ${len(possible_pins)}'b10;
    end else begin
      if (reg_we & ${block_input}_${inst}${bit_str}_sel_addressed) begin
        ${block_input}_${inst}${bit_str}_sel <= reg_wdata[${(input_idx%4)*8}+:${len(possible_pins)}];
      end
    end
  end

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
  % for block_input, inst, combine_pins, combine_pin_selectors, combine_type in combine_list:
  assign ${block_input}_o[${inst}] =
    % for idx, pin in enumerate(combine_pins):
    (${pin}_sel == ${combine_pin_selectors[idx]} ? ${pin} : ${'1' if combine_type == 'and' else '0'})${';' if idx == len(combine_pins)-1 else ' &' if combine_type == 'and' else ' |'}
    % endfor
  % endfor
endmodule
