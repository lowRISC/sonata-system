// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Hardware module that gives information about the system.
// For example the commit hash from which this bitstream was generated.
// This file has been auto-generated, please edit the file in the rtl/templates folder.
// Please do not commit a generated SystemVerilog version of this file with an actual git hash.
// The value in the git hash commited to the repository should be equal to all zeroes.
// Only when generating a release, the top generator can be used to insert the final hash into the system.
// To do this use the command:
// ./util/top_gen.py system_info

module system_info import system_info_reg_pkg::*; #(
  parameter int unsigned SysClkFreq = 0,
  parameter int unsigned GpioNum    = 0,
  parameter int unsigned UartNum    = 0,
  parameter int unsigned I2cNum     = 0,
  parameter int unsigned SpiNum     = 0
) (
  // Clock and reset.
  input logic clk_i,
  input logic rst_ni,

  // TileLink interfaces.
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);
  system_info_hw2reg_t hw2reg;

  assign hw2reg.rtl_commit_hash_0.d = 'h${system_info.commit_hash[0:8]};
  assign hw2reg.rtl_commit_hash_1.d = 'h${system_info.commit_hash[8:16]};
  assign hw2reg.rtl_commit_dirty.d = 'b${'1' if system_info.dirty else '0'};
  assign hw2reg.system_frequency.d = SysClkFreq;
  assign hw2reg.gpio_info.d = 8'(GpioNum);
  assign hw2reg.uart_info.d = 8'(UartNum);
  assign hw2reg.i2c_info.d = 8'(I2cNum);
  assign hw2reg.spi_info.d = 8'(SpiNum);

  // Instantiate the registers
  system_info_reg_top u_system_info_reg_top (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .hw2reg
  );
endmodule
