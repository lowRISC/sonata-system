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

endpackage : sonata_pkg
