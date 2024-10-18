// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// sonata package

package sonata_pkg;

  localparam int unsigned GPIO_NUM = ${gpio_num};
  localparam int unsigned UART_NUM = ${uart_num};
  localparam int unsigned I2C_NUM  = ${i2c_num};
  localparam int unsigned SPI_NUM  = ${spi_num};

  localparam int unsigned SPI_CS_NUM = 4;

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
