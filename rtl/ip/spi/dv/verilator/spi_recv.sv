/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
module spi_recv #(parameter bit CPHA = 1'b0) (
  input spi_clk,
  input spi_copi_i
);
  logic [7:0] data_in;
  logic [2:0] bit_count;

  bit [7:0] data_recv[$];

  if (CPHA) begin : g_positive_cpha
    always @(negedge spi_clk) begin
      data_in <= {data_in[6:0], spi_copi_i};
      bit_count <= bit_count + 3'd1;

      if (bit_count == 3'd7) begin
        data_recv.push_back({data_in[6:0], spi_copi_i});
      end
    end
  end else begin : g_negative_cpha
    always @(posedge spi_clk) begin
      data_in <= {data_in[6:0], spi_copi_i};
      bit_count <= bit_count + 3'd1;

      if (bit_count == 3'd7) begin
        data_recv.push_back({data_in[6:0], spi_copi_i});
      end
    end
  end
endmodule
