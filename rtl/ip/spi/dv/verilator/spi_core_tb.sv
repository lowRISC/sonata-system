/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
module spi_core_tb;
  logic clk, rst_n;

  localparam int unsigned TotalBytes = 300;
  localparam int unsigned LargestXfer = 16;

  initial begin
    $dumpfile("sim.fst");
    $dumpvars;
    $dumpon;

    rst_n = 1'b0;
    clk = 1'b0;
    #3 rst_n = 1'b1;
    #2 rst_n = 1'b0;
    #3 rst_n = 1'b1;
    forever begin
      #5 clk = ~clk;
    end
  end

  bit [7:0] data_out_exp[$];
  bit [7:0] data_out_seen[$];

  initial begin
    for (int i = 0;i < TotalBytes; ++i) begin
      bit [7:0] rnd_byte;

      rnd_byte = 8'($random());
      u_trans.data_transmit.push_back(rnd_byte);
      data_out_exp.push_back(rnd_byte);
    end
  end

  logic [7:0]  data_in;
  logic        data_in_ready;
  logic        data_in_valid;
  logic [31:0] data_sent_count;
  logic [7:0]  valid_countdown;
  logic        rnd_start;
  logic [10:0] byte_count;

  logic [31:0] data_start_count;

  logic [7:0] data_sent[$];

  logic spi_idle;

  always @(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
      data_in          <= 8'($random());
      data_sent_count  <= '0;
      valid_countdown  <= 8'($random());
      data_in_valid    <= 1'b0;
      rnd_start        <= 1'($random());
      data_start_count <= '0;
      byte_count       <= 11'($urandom_range(1, LargestXfer));
    end else begin
      rnd_start <= 1'($random());

      if (!data_in_valid && data_sent_count < TotalBytes) begin
        if (valid_countdown == '0) begin
          data_in_valid <= 1'b1;
        end else begin
          valid_countdown <= valid_countdown - 8'd1;
        end
      end

      if (data_in_valid && data_in_ready) begin
        data_in    <= 8'($random());
        data_sent_count <= data_sent_count + 32'd1;
        data_sent.push_back(data_in);

        if (data_sent_count == TotalBytes - 1) begin
          data_in_valid <= 1'b0;
        end else if (1'($random()) == 1'b0) begin
          valid_countdown <= 8'($random());
          data_in_valid <= 1'b0;
        end
      end

      if (rnd_start && spi_idle) begin
        data_start_count <= data_start_count + 32'(byte_count);

        if ((TotalBytes - (32'(byte_count) + data_start_count)) < LargestXfer) begin
          byte_count <= 11'($urandom_range(1, (TotalBytes - (32'(byte_count) + data_start_count))));
        end else begin
          byte_count <= 11'($urandom_range(1, LargestXfer));
        end
      end
    end
  end

  always @(posedge clk) begin
    if (data_sent_count == TotalBytes && spi_idle) begin
      foreach(u_recv.data_recv[i]) begin
        if (i < data_sent.size()) begin
          if (u_recv.data_recv[i] != data_sent[i]) begin
            $display("ERROR: Mismatch at %d for receive byte sent: %02X, recv: %02X", i, data_sent[i],
              u_recv.data_recv[i]);
          end else begin
            $display("Receive byte %d: %02X", i, data_sent[i]);
          end
        end else begin
          $display("ERROR: Not enough sent data, reached %d", i);
        end
      end

      foreach(data_out_exp[i]) begin
        if (i < data_out_seen.size()) begin
          if (data_out_seen[i] != data_out_exp[i]) begin
            $display("ERROR: Mismatch at %d for transmit byte, expected: %02X, seen: %02X", i, data_out_exp[i],
              data_out_seen[i]);
          end else begin
            $display("Transmit byte %d: %02X", i, data_out_exp[i]);
          end
        end else begin
          $display("ERROR: Not enough received data, reached %d", i);
        end
      end

      $finish();
    end
  end


  logic [7:0] data_out;
  logic data_out_valid, data_out_ready;
  logic [7:0] ready_countdown;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ready_countdown <= 8'($random());
    end else begin
      if (data_out_ready && data_out_valid) begin
        ready_countdown <= 8'($random());
        data_out_seen.push_back(data_out);
      end else if (ready_countdown != '0) begin
        ready_countdown <= ready_countdown - 8'd1;
      end
    end
  end

  assign data_out_ready = ready_countdown == '0;

  logic spi_clk;
  logic spi_copi;
  logic spi_cipo;

  spi_core u_dut (
    .clk_i(clk),
    .rst_ni(rst_n),

    .data_in_i      (data_in),
    .data_in_valid_i(data_in_valid),
    .data_in_ready_o(data_in_ready),

    .data_out_o      (data_out),
    .data_out_valid_o(data_out_valid),
    .data_out_ready_i(data_out_ready),

    .start_i     (rnd_start),
    .byte_count_i(byte_count),
    .idle_o      (spi_idle),

    .cpol_i     (1'b0),
    .cpha_i     (1'b1),
    .msb_first_i(1'b1),

    .spi_copi_o(spi_copi),
    .spi_cipo_i(spi_cipo),
    .spi_clk_o (spi_clk),

    .half_clk_period_i(16'd5)
  );

  spi_recv #(.CPHA(1'b1)) u_recv (
    .spi_clk(spi_clk),
    .spi_copi_i(spi_copi)
  );

  spi_trans #(.CPHA(1'b1)) u_trans (
    .rst_ni(rst_n),
    .spi_clk(spi_clk),
    .spi_cipo_o(spi_cipo)
  );
endmodule
