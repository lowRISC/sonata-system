// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// sonata package

package sonata_pkg;

  // Number of Instances
  % for block in config.blocks:
  localparam int unsigned ${block.name.upper()}_NUM = ${block.instances};
  % endfor

  // Width of block IO arrays
  % for block in config.blocks:
  % for b_io in block.ios:
  % if b_io.length is not None:
  localparam int unsigned ${f"{block.name}_{b_io.name}_WIDTH".upper()} = ${b_io.length};
  % endif
  % endfor
  % endfor

  // Instance-specific GPIO core input/output widths.
  // Include the fixed (non-pinmux) GPIO used for on-board peripherals.
  // Each must be less than GPIO_IOS_WIDTH.
  localparam int unsigned GPIO_INST_IN_WIDTH[GPIO_NUM]  = {17, 28, 14, 8, 8, 6};
  localparam int unsigned GPIO_INST_OUT_WIDTH[GPIO_NUM] = { 8, 28, 14, 8, 8, 6};

  // Number of input, output, and inout pins
  localparam int unsigned IN_PIN_NUM = ${len(in_pins)};
  localparam int unsigned OUT_PIN_NUM = ${len(out_pins)};
  localparam int unsigned INOUT_PIN_NUM = ${len(inout_pins)};

  % for in_pin_index, pin in enumerate(in_pins):
  localparam int unsigned ${pin.idx_param} = ${in_pin_index};
  % endfor

  % for out_pin_index, pin in enumerate(out_pins):
  localparam int unsigned ${pin.idx_param} = ${out_pin_index};
  % endfor

  % for inout_pin_index, pin in enumerate(inout_pins):
  localparam int unsigned ${pin.idx_param} = ${inout_pin_index};
  % endfor

  typedef logic [   IN_PIN_NUM-1:0] sonata_in_pins_t;
  typedef logic [  OUT_PIN_NUM-1:0] sonata_out_pins_t;
  typedef logic [INOUT_PIN_NUM-1:0] sonata_inout_pins_t;

endpackage : sonata_pkg
