/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
module spi_trans #(parameter bit CPHA = 1'b0) (
  input  rst_ni,
  input  spi_clk,
  output spi_cipo_o
);
  logic [7:0] data_out;
  logic [2:0] bit_count;

  bit [7:0] data_transmit[$];

  if (CPHA) begin : g_positive_cpha
    always @(posedge spi_clk or negedge rst_ni) begin
      if (!rst_ni) begin
        data_out  <= '0;
        bit_count <= '0;
      end else if (bit_count == 3'b0) begin
        if (data_transmit.size() != 0) begin
          data_out <= data_transmit.pop_front();
        end else begin
          data_out <= '0;
        end
        bit_count <= 3'd7;
      end else begin
        bit_count <= bit_count - 3'b1;
        data_out  <= {data_out[6:0], 1'b0};
      end
    end
  end else begin : g_negative_cpha
    always @(negedge spi_clk or negedge rst_ni) begin
      if (!rst_ni) begin
        if (data_transmit.size() != 0) begin
            data_out <= data_transmit.pop_front();
        end else begin
            data_out <= '0;
        end

        bit_count <= 3'd7;
      end else if (bit_count == 3'b0) begin
        if (data_transmit.size() != 0) begin
          data_out <= data_transmit.pop_front();
        end else begin
          data_out <= '0;
        end
        bit_count <= 3'd7;
      end else begin
        bit_count <= bit_count - 3'b1;
        data_out  <= {data_out[6:0], 1'b0};
      end
    end
  end

  assign spi_cipo_o = data_out[7];
endmodule
