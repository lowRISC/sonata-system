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
  input  logic uart_tx_i[5],
  output logic uart_rx_o[5],
  input  logic i2c_scl_i[2],
  input  logic i2c_scl_en_i[2],
  output logic i2c_scl_o[2],
  input  logic i2c_sda_i[2],
  input  logic i2c_sda_en_i[2],
  output logic i2c_sda_o[2],
  input  logic spi_sck_i[5],
  input  logic spi_tx_i[5],
  output logic spi_rx_o[5],
  input  logic [31:0] gpio_ios_i[3],
  input  logic [31:0] gpio_ios_en_i[3],
  output logic [31:0] gpio_ios_o[3],

  // List of pins.
  inout logic ser0_tx,
  inout logic ser0_rx,
  inout logic ser1_tx,
  inout logic ser1_rx,
  inout logic rs232_tx,
  inout logic rs232_rx,
  inout logic scl0,
  inout logic sda0,
  inout logic scl1,
  inout logic sda1,
  inout logic appspi_d0,
  inout logic appspi_d1,
  inout logic appspi_clk,
  inout logic lcd_copi,
  inout logic lcd_clk,
  inout logic ethmac_copi,
  inout logic ethmac_cipo,
  inout logic ethmac_sclk,
  inout logic rph_g0,
  inout logic rph_g1,
  inout logic rph_g2_sda,
  inout logic rph_g3_scl,
  inout logic rph_g4,
  inout logic rph_g5,
  inout logic rph_g6,
  inout logic rph_g7_ce1,
  inout logic rph_g8_ce0,
  inout logic rph_g9_cipo,
  inout logic rph_g10_copi,
  inout logic rph_g11_sclk,
  inout logic rph_g12,
  inout logic rph_g13,
  inout logic rph_txd0,
  inout logic rph_rxd0,
  inout logic rph_g16_ce2,
  inout logic rph_g17,
  inout logic rph_g18,
  inout logic rph_g19_cipo,
  inout logic rph_g20_copi,
  inout logic rph_g21_sclk,
  inout logic rph_g22,
  inout logic rph_g23,
  inout logic rph_g24,
  inout logic rph_g25,
  inout logic rph_g26,
  inout logic rph_g27,
  inout logic ah_tmpio0,
  inout logic ah_tmpio1,
  inout logic ah_tmpio2,
  inout logic ah_tmpio3,
  inout logic ah_tmpio4,
  inout logic ah_tmpio5,
  inout logic ah_tmpio6,
  inout logic ah_tmpio7,
  inout logic ah_tmpio8,
  inout logic ah_tmpio9,
  inout logic ah_tmpio10,
  inout logic ah_tmpio11,
  inout logic ah_tmpio12,
  inout logic ah_tmpio13,
  inout logic ah_tmpio14,
  inout logic ah_tmpio15,
  inout logic ah_tmpio16,
  inout logic ah_tmpio17,
  inout logic mb2,
  inout logic mb3,
  inout logic mb4,
  inout logic mb5,
  inout logic mb6,
  inout logic mb7,
  inout logic mb8,
  inout logic [7:0] pmod0,
  inout logic [7:0] pmod1,

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
    .EnableRspIntgGen ( 1            ),
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

  logic [1:0] ser0_tx_sel;
  logic ser0_tx_o;
  logic ser0_tx_en_o;
  logic ser0_tx_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ser0_tx_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ser0_tx_sel <= 2'b10;
    end else begin
      if (reg_we & ser0_tx_sel_addressed) begin
        ser0_tx_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ser0_tx_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      uart_tx_i[0]
    }),
    .sel_i(ser0_tx_sel),
    .out_o(ser0_tx_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ser0_tx_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(ser0_tx_sel),
    .out_o(ser0_tx_en_o)
  );

  assign ser0_tx = ser0_tx_en_o ? ser0_tx_o : 1'bz;

  logic [1:0] ser1_tx_sel;
  logic ser1_tx_o;
  logic ser1_tx_en_o;
  logic ser1_tx_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ser1_tx_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ser1_tx_sel <= 2'b10;
    end else begin
      if (reg_we & ser1_tx_sel_addressed) begin
        ser1_tx_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ser1_tx_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      uart_tx_i[1]
    }),
    .sel_i(ser1_tx_sel),
    .out_o(ser1_tx_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ser1_tx_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(ser1_tx_sel),
    .out_o(ser1_tx_en_o)
  );

  assign ser1_tx = ser1_tx_en_o ? ser1_tx_o : 1'bz;

  logic [1:0] rs232_tx_sel;
  logic rs232_tx_o;
  logic rs232_tx_en_o;
  logic rs232_tx_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rs232_tx_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rs232_tx_sel <= 2'b10;
    end else begin
      if (reg_we & rs232_tx_sel_addressed) begin
        rs232_tx_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rs232_tx_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      uart_tx_i[4]
    }),
    .sel_i(rs232_tx_sel),
    .out_o(rs232_tx_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rs232_tx_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(rs232_tx_sel),
    .out_o(rs232_tx_en_o)
  );

  assign rs232_tx = rs232_tx_en_o ? rs232_tx_o : 1'bz;

  logic [1:0] scl0_sel;
  logic scl0_o;
  logic scl0_en_o;
  logic scl0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign scl0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      scl0_sel <= 2'b10;
    end else begin
      if (reg_we & scl0_sel_addressed) begin
        scl0_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) scl0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_scl_i[0]
    }),
    .sel_i(scl0_sel),
    .out_o(scl0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) scl0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_scl_en_i[0]
    }),
    .sel_i(scl0_sel),
    .out_o(scl0_en_o)
  );

  assign scl0 = scl0_en_o ? scl0_o : 1'bz;

  logic [1:0] sda0_sel;
  logic sda0_o;
  logic sda0_en_o;
  logic sda0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign sda0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      sda0_sel <= 2'b10;
    end else begin
      if (reg_we & sda0_sel_addressed) begin
        sda0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) sda0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_sda_i[0]
    }),
    .sel_i(sda0_sel),
    .out_o(sda0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) sda0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_sda_en_i[0]
    }),
    .sel_i(sda0_sel),
    .out_o(sda0_en_o)
  );

  assign sda0 = sda0_en_o ? sda0_o : 1'bz;

  logic [1:0] scl1_sel;
  logic scl1_o;
  logic scl1_en_o;
  logic scl1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign scl1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      scl1_sel <= 2'b10;
    end else begin
      if (reg_we & scl1_sel_addressed) begin
        scl1_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) scl1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_scl_i[1]
    }),
    .sel_i(scl1_sel),
    .out_o(scl1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) scl1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_scl_en_i[1]
    }),
    .sel_i(scl1_sel),
    .out_o(scl1_en_o)
  );

  assign scl1 = scl1_en_o ? scl1_o : 1'bz;

  logic [1:0] sda1_sel;
  logic sda1_o;
  logic sda1_en_o;
  logic sda1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign sda1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      sda1_sel <= 2'b10;
    end else begin
      if (reg_we & sda1_sel_addressed) begin
        sda1_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) sda1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_sda_i[1]
    }),
    .sel_i(sda1_sel),
    .out_o(sda1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) sda1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_sda_en_i[1]
    }),
    .sel_i(sda1_sel),
    .out_o(sda1_en_o)
  );

  assign sda1 = sda1_en_o ? sda1_o : 1'bz;

  logic [1:0] appspi_d0_sel;
  logic appspi_d0_o;
  logic appspi_d0_en_o;
  logic appspi_d0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign appspi_d0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      appspi_d0_sel <= 2'b10;
    end else begin
      if (reg_we & appspi_d0_sel_addressed) begin
        appspi_d0_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) appspi_d0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[0]
    }),
    .sel_i(appspi_d0_sel),
    .out_o(appspi_d0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) appspi_d0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(appspi_d0_sel),
    .out_o(appspi_d0_en_o)
  );

  assign appspi_d0 = appspi_d0_en_o ? appspi_d0_o : 1'bz;

  logic [1:0] appspi_clk_sel;
  logic appspi_clk_o;
  logic appspi_clk_en_o;
  logic appspi_clk_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign appspi_clk_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      appspi_clk_sel <= 2'b10;
    end else begin
      if (reg_we & appspi_clk_sel_addressed) begin
        appspi_clk_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) appspi_clk_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[0]
    }),
    .sel_i(appspi_clk_sel),
    .out_o(appspi_clk_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) appspi_clk_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(appspi_clk_sel),
    .out_o(appspi_clk_en_o)
  );

  assign appspi_clk = appspi_clk_en_o ? appspi_clk_o : 1'bz;

  logic [1:0] lcd_copi_sel;
  logic lcd_copi_o;
  logic lcd_copi_en_o;
  logic lcd_copi_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign lcd_copi_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      lcd_copi_sel <= 2'b10;
    end else begin
      if (reg_we & lcd_copi_sel_addressed) begin
        lcd_copi_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) lcd_copi_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[1]
    }),
    .sel_i(lcd_copi_sel),
    .out_o(lcd_copi_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) lcd_copi_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(lcd_copi_sel),
    .out_o(lcd_copi_en_o)
  );

  assign lcd_copi = lcd_copi_en_o ? lcd_copi_o : 1'bz;

  logic [1:0] lcd_clk_sel;
  logic lcd_clk_o;
  logic lcd_clk_en_o;
  logic lcd_clk_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign lcd_clk_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      lcd_clk_sel <= 2'b10;
    end else begin
      if (reg_we & lcd_clk_sel_addressed) begin
        lcd_clk_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) lcd_clk_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[1]
    }),
    .sel_i(lcd_clk_sel),
    .out_o(lcd_clk_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) lcd_clk_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(lcd_clk_sel),
    .out_o(lcd_clk_en_o)
  );

  assign lcd_clk = lcd_clk_en_o ? lcd_clk_o : 1'bz;

  logic [1:0] ethmac_copi_sel;
  logic ethmac_copi_o;
  logic ethmac_copi_en_o;
  logic ethmac_copi_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ethmac_copi_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ethmac_copi_sel <= 2'b10;
    end else begin
      if (reg_we & ethmac_copi_sel_addressed) begin
        ethmac_copi_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ethmac_copi_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[2]
    }),
    .sel_i(ethmac_copi_sel),
    .out_o(ethmac_copi_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ethmac_copi_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(ethmac_copi_sel),
    .out_o(ethmac_copi_en_o)
  );

  assign ethmac_copi = ethmac_copi_en_o ? ethmac_copi_o : 1'bz;

  logic [1:0] ethmac_sclk_sel;
  logic ethmac_sclk_o;
  logic ethmac_sclk_en_o;
  logic ethmac_sclk_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ethmac_sclk_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ethmac_sclk_sel <= 2'b10;
    end else begin
      if (reg_we & ethmac_sclk_sel_addressed) begin
        ethmac_sclk_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ethmac_sclk_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[2]
    }),
    .sel_i(ethmac_sclk_sel),
    .out_o(ethmac_sclk_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ethmac_sclk_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(ethmac_sclk_sel),
    .out_o(ethmac_sclk_en_o)
  );

  assign ethmac_sclk = ethmac_sclk_en_o ? ethmac_sclk_o : 1'bz;

  logic [2:0] rph_g0_sel;
  logic rph_g0_o;
  logic rph_g0_en_o;
  logic rph_g0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g0_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g0_sel_addressed) begin
        rph_g0_sel <= reg_wdata[8+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_sda_i[0],
      gpio_ios_i[0][0]
    }),
    .sel_i(rph_g0_sel),
    .out_o(rph_g0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_sda_en_i[0],
      gpio_ios_en_i[0][0]
    }),
    .sel_i(rph_g0_sel),
    .out_o(rph_g0_en_o)
  );

  assign rph_g0 = rph_g0_en_o ? rph_g0_o : 1'bz;

  logic [2:0] rph_g1_sel;
  logic rph_g1_o;
  logic rph_g1_en_o;
  logic rph_g1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g1_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g1_sel_addressed) begin
        rph_g1_sel <= reg_wdata[16+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_scl_i[0],
      gpio_ios_i[0][1]
    }),
    .sel_i(rph_g1_sel),
    .out_o(rph_g1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_scl_en_i[0],
      gpio_ios_en_i[0][1]
    }),
    .sel_i(rph_g1_sel),
    .out_o(rph_g1_en_o)
  );

  assign rph_g1 = rph_g1_en_o ? rph_g1_o : 1'bz;

  logic [2:0] rph_g2_sda_sel;
  logic rph_g2_sda_o;
  logic rph_g2_sda_en_o;
  logic rph_g2_sda_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g2_sda_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g2_sda_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g2_sda_sel_addressed) begin
        rph_g2_sda_sel <= reg_wdata[24+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g2_sda_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_sda_i[1],
      gpio_ios_i[0][2]
    }),
    .sel_i(rph_g2_sda_sel),
    .out_o(rph_g2_sda_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g2_sda_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_sda_en_i[1],
      gpio_ios_en_i[0][2]
    }),
    .sel_i(rph_g2_sda_sel),
    .out_o(rph_g2_sda_en_o)
  );

  assign rph_g2_sda = rph_g2_sda_en_o ? rph_g2_sda_o : 1'bz;

  logic [2:0] rph_g3_scl_sel;
  logic rph_g3_scl_o;
  logic rph_g3_scl_en_o;
  logic rph_g3_scl_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g3_scl_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g3_scl_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g3_scl_sel_addressed) begin
        rph_g3_scl_sel <= reg_wdata[0+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g3_scl_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_scl_i[1],
      gpio_ios_i[0][3]
    }),
    .sel_i(rph_g3_scl_sel),
    .out_o(rph_g3_scl_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g3_scl_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_scl_en_i[1],
      gpio_ios_en_i[0][3]
    }),
    .sel_i(rph_g3_scl_sel),
    .out_o(rph_g3_scl_en_o)
  );

  assign rph_g3_scl = rph_g3_scl_en_o ? rph_g3_scl_o : 1'bz;

  logic [1:0] rph_g4_sel;
  logic rph_g4_o;
  logic rph_g4_en_o;
  logic rph_g4_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g4_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g4_sel_addressed) begin
        rph_g4_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][4]
    }),
    .sel_i(rph_g4_sel),
    .out_o(rph_g4_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g4_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][4]
    }),
    .sel_i(rph_g4_sel),
    .out_o(rph_g4_en_o)
  );

  assign rph_g4 = rph_g4_en_o ? rph_g4_o : 1'bz;

  logic [1:0] rph_g5_sel;
  logic rph_g5_o;
  logic rph_g5_en_o;
  logic rph_g5_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g5_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g5_sel_addressed) begin
        rph_g5_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][5]
    }),
    .sel_i(rph_g5_sel),
    .out_o(rph_g5_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g5_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][5]
    }),
    .sel_i(rph_g5_sel),
    .out_o(rph_g5_en_o)
  );

  assign rph_g5 = rph_g5_en_o ? rph_g5_o : 1'bz;

  logic [1:0] rph_g6_sel;
  logic rph_g6_o;
  logic rph_g6_en_o;
  logic rph_g6_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g6_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g6_sel_addressed) begin
        rph_g6_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][6]
    }),
    .sel_i(rph_g6_sel),
    .out_o(rph_g6_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g6_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][6]
    }),
    .sel_i(rph_g6_sel),
    .out_o(rph_g6_en_o)
  );

  assign rph_g6 = rph_g6_en_o ? rph_g6_o : 1'bz;

  logic [1:0] rph_g7_ce1_sel;
  logic rph_g7_ce1_o;
  logic rph_g7_ce1_en_o;
  logic rph_g7_ce1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g7_ce1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g7_ce1_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g7_ce1_sel_addressed) begin
        rph_g7_ce1_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g7_ce1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][7]
    }),
    .sel_i(rph_g7_ce1_sel),
    .out_o(rph_g7_ce1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g7_ce1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][7]
    }),
    .sel_i(rph_g7_ce1_sel),
    .out_o(rph_g7_ce1_en_o)
  );

  assign rph_g7_ce1 = rph_g7_ce1_en_o ? rph_g7_ce1_o : 1'bz;

  logic [1:0] rph_g8_ce0_sel;
  logic rph_g8_ce0_o;
  logic rph_g8_ce0_en_o;
  logic rph_g8_ce0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g8_ce0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g8_ce0_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g8_ce0_sel_addressed) begin
        rph_g8_ce0_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g8_ce0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][8]
    }),
    .sel_i(rph_g8_ce0_sel),
    .out_o(rph_g8_ce0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g8_ce0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][8]
    }),
    .sel_i(rph_g8_ce0_sel),
    .out_o(rph_g8_ce0_en_o)
  );

  assign rph_g8_ce0 = rph_g8_ce0_en_o ? rph_g8_ce0_o : 1'bz;

  logic [1:0] rph_g9_cipo_sel;
  logic rph_g9_cipo_o;
  logic rph_g9_cipo_en_o;
  logic rph_g9_cipo_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g9_cipo_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g9_cipo_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g9_cipo_sel_addressed) begin
        rph_g9_cipo_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g9_cipo_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][9]
    }),
    .sel_i(rph_g9_cipo_sel),
    .out_o(rph_g9_cipo_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g9_cipo_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][9]
    }),
    .sel_i(rph_g9_cipo_sel),
    .out_o(rph_g9_cipo_en_o)
  );

  assign rph_g9_cipo = rph_g9_cipo_en_o ? rph_g9_cipo_o : 1'bz;

  logic [2:0] rph_g10_copi_sel;
  logic rph_g10_copi_o;
  logic rph_g10_copi_en_o;
  logic rph_g10_copi_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g10_copi_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g10_copi_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g10_copi_sel_addressed) begin
        rph_g10_copi_sel <= reg_wdata[24+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g10_copi_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[3],
      gpio_ios_i[0][10]
    }),
    .sel_i(rph_g10_copi_sel),
    .out_o(rph_g10_copi_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g10_copi_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[0][10]
    }),
    .sel_i(rph_g10_copi_sel),
    .out_o(rph_g10_copi_en_o)
  );

  assign rph_g10_copi = rph_g10_copi_en_o ? rph_g10_copi_o : 1'bz;

  logic [2:0] rph_g11_sclk_sel;
  logic rph_g11_sclk_o;
  logic rph_g11_sclk_en_o;
  logic rph_g11_sclk_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g11_sclk_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g11_sclk_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g11_sclk_sel_addressed) begin
        rph_g11_sclk_sel <= reg_wdata[0+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g11_sclk_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[3],
      gpio_ios_i[0][11]
    }),
    .sel_i(rph_g11_sclk_sel),
    .out_o(rph_g11_sclk_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g11_sclk_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[0][11]
    }),
    .sel_i(rph_g11_sclk_sel),
    .out_o(rph_g11_sclk_en_o)
  );

  assign rph_g11_sclk = rph_g11_sclk_en_o ? rph_g11_sclk_o : 1'bz;

  logic [1:0] rph_g12_sel;
  logic rph_g12_o;
  logic rph_g12_en_o;
  logic rph_g12_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g12_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g12_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g12_sel_addressed) begin
        rph_g12_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g12_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][12]
    }),
    .sel_i(rph_g12_sel),
    .out_o(rph_g12_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g12_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][12]
    }),
    .sel_i(rph_g12_sel),
    .out_o(rph_g12_en_o)
  );

  assign rph_g12 = rph_g12_en_o ? rph_g12_o : 1'bz;

  logic [1:0] rph_g13_sel;
  logic rph_g13_o;
  logic rph_g13_en_o;
  logic rph_g13_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g13_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g13_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g13_sel_addressed) begin
        rph_g13_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g13_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][13]
    }),
    .sel_i(rph_g13_sel),
    .out_o(rph_g13_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g13_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][13]
    }),
    .sel_i(rph_g13_sel),
    .out_o(rph_g13_en_o)
  );

  assign rph_g13 = rph_g13_en_o ? rph_g13_o : 1'bz;

  logic [2:0] rph_txd0_sel;
  logic rph_txd0_o;
  logic rph_txd0_en_o;
  logic rph_txd0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_txd0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_txd0_sel <= 3'b10;
    end else begin
      if (reg_we & rph_txd0_sel_addressed) begin
        rph_txd0_sel <= reg_wdata[24+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_txd0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      uart_tx_i[2],
      gpio_ios_i[0][14]
    }),
    .sel_i(rph_txd0_sel),
    .out_o(rph_txd0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_txd0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[0][14]
    }),
    .sel_i(rph_txd0_sel),
    .out_o(rph_txd0_en_o)
  );

  assign rph_txd0 = rph_txd0_en_o ? rph_txd0_o : 1'bz;

  logic [1:0] rph_rxd0_sel;
  logic rph_rxd0_o;
  logic rph_rxd0_en_o;
  logic rph_rxd0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_rxd0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_rxd0_sel <= 2'b10;
    end else begin
      if (reg_we & rph_rxd0_sel_addressed) begin
        rph_rxd0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_rxd0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][15]
    }),
    .sel_i(rph_rxd0_sel),
    .out_o(rph_rxd0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_rxd0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][15]
    }),
    .sel_i(rph_rxd0_sel),
    .out_o(rph_rxd0_en_o)
  );

  assign rph_rxd0 = rph_rxd0_en_o ? rph_rxd0_o : 1'bz;

  logic [1:0] rph_g16_ce2_sel;
  logic rph_g16_ce2_o;
  logic rph_g16_ce2_en_o;
  logic rph_g16_ce2_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g16_ce2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g16_ce2_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g16_ce2_sel_addressed) begin
        rph_g16_ce2_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g16_ce2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][16]
    }),
    .sel_i(rph_g16_ce2_sel),
    .out_o(rph_g16_ce2_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g16_ce2_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][16]
    }),
    .sel_i(rph_g16_ce2_sel),
    .out_o(rph_g16_ce2_en_o)
  );

  assign rph_g16_ce2 = rph_g16_ce2_en_o ? rph_g16_ce2_o : 1'bz;

  logic [1:0] rph_g17_sel;
  logic rph_g17_o;
  logic rph_g17_en_o;
  logic rph_g17_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g17_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g17_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g17_sel_addressed) begin
        rph_g17_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g17_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][17]
    }),
    .sel_i(rph_g17_sel),
    .out_o(rph_g17_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g17_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][17]
    }),
    .sel_i(rph_g17_sel),
    .out_o(rph_g17_en_o)
  );

  assign rph_g17 = rph_g17_en_o ? rph_g17_o : 1'bz;

  logic [1:0] rph_g18_sel;
  logic rph_g18_o;
  logic rph_g18_en_o;
  logic rph_g18_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g18_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g18_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g18_sel_addressed) begin
        rph_g18_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g18_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][18]
    }),
    .sel_i(rph_g18_sel),
    .out_o(rph_g18_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g18_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][18]
    }),
    .sel_i(rph_g18_sel),
    .out_o(rph_g18_en_o)
  );

  assign rph_g18 = rph_g18_en_o ? rph_g18_o : 1'bz;

  logic [1:0] rph_g19_cipo_sel;
  logic rph_g19_cipo_o;
  logic rph_g19_cipo_en_o;
  logic rph_g19_cipo_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g19_cipo_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g19_cipo_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g19_cipo_sel_addressed) begin
        rph_g19_cipo_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g19_cipo_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][19]
    }),
    .sel_i(rph_g19_cipo_sel),
    .out_o(rph_g19_cipo_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g19_cipo_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][19]
    }),
    .sel_i(rph_g19_cipo_sel),
    .out_o(rph_g19_cipo_en_o)
  );

  assign rph_g19_cipo = rph_g19_cipo_en_o ? rph_g19_cipo_o : 1'bz;

  logic [2:0] rph_g20_copi_sel;
  logic rph_g20_copi_o;
  logic rph_g20_copi_en_o;
  logic rph_g20_copi_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g20_copi_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g20_copi_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g20_copi_sel_addressed) begin
        rph_g20_copi_sel <= reg_wdata[8+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g20_copi_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[4],
      gpio_ios_i[0][20]
    }),
    .sel_i(rph_g20_copi_sel),
    .out_o(rph_g20_copi_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g20_copi_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[0][20]
    }),
    .sel_i(rph_g20_copi_sel),
    .out_o(rph_g20_copi_en_o)
  );

  assign rph_g20_copi = rph_g20_copi_en_o ? rph_g20_copi_o : 1'bz;

  logic [2:0] rph_g21_sclk_sel;
  logic rph_g21_sclk_o;
  logic rph_g21_sclk_en_o;
  logic rph_g21_sclk_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g21_sclk_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g21_sclk_sel <= 3'b10;
    end else begin
      if (reg_we & rph_g21_sclk_sel_addressed) begin
        rph_g21_sclk_sel <= reg_wdata[16+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g21_sclk_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[4],
      gpio_ios_i[0][21]
    }),
    .sel_i(rph_g21_sclk_sel),
    .out_o(rph_g21_sclk_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) rph_g21_sclk_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[0][21]
    }),
    .sel_i(rph_g21_sclk_sel),
    .out_o(rph_g21_sclk_en_o)
  );

  assign rph_g21_sclk = rph_g21_sclk_en_o ? rph_g21_sclk_o : 1'bz;

  logic [1:0] rph_g22_sel;
  logic rph_g22_o;
  logic rph_g22_en_o;
  logic rph_g22_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g22_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g22_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g22_sel_addressed) begin
        rph_g22_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g22_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][22]
    }),
    .sel_i(rph_g22_sel),
    .out_o(rph_g22_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g22_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][22]
    }),
    .sel_i(rph_g22_sel),
    .out_o(rph_g22_en_o)
  );

  assign rph_g22 = rph_g22_en_o ? rph_g22_o : 1'bz;

  logic [1:0] rph_g23_sel;
  logic rph_g23_o;
  logic rph_g23_en_o;
  logic rph_g23_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g23_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g23_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g23_sel_addressed) begin
        rph_g23_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g23_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][23]
    }),
    .sel_i(rph_g23_sel),
    .out_o(rph_g23_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g23_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][23]
    }),
    .sel_i(rph_g23_sel),
    .out_o(rph_g23_en_o)
  );

  assign rph_g23 = rph_g23_en_o ? rph_g23_o : 1'bz;

  logic [1:0] rph_g24_sel;
  logic rph_g24_o;
  logic rph_g24_en_o;
  logic rph_g24_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g24_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g24_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g24_sel_addressed) begin
        rph_g24_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g24_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][24]
    }),
    .sel_i(rph_g24_sel),
    .out_o(rph_g24_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g24_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][24]
    }),
    .sel_i(rph_g24_sel),
    .out_o(rph_g24_en_o)
  );

  assign rph_g24 = rph_g24_en_o ? rph_g24_o : 1'bz;

  logic [1:0] rph_g25_sel;
  logic rph_g25_o;
  logic rph_g25_en_o;
  logic rph_g25_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g25_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g25_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g25_sel_addressed) begin
        rph_g25_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g25_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][25]
    }),
    .sel_i(rph_g25_sel),
    .out_o(rph_g25_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g25_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][25]
    }),
    .sel_i(rph_g25_sel),
    .out_o(rph_g25_en_o)
  );

  assign rph_g25 = rph_g25_en_o ? rph_g25_o : 1'bz;

  logic [1:0] rph_g26_sel;
  logic rph_g26_o;
  logic rph_g26_en_o;
  logic rph_g26_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g26_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g26_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g26_sel_addressed) begin
        rph_g26_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g26_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][26]
    }),
    .sel_i(rph_g26_sel),
    .out_o(rph_g26_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g26_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][26]
    }),
    .sel_i(rph_g26_sel),
    .out_o(rph_g26_en_o)
  );

  assign rph_g26 = rph_g26_en_o ? rph_g26_o : 1'bz;

  logic [1:0] rph_g27_sel;
  logic rph_g27_o;
  logic rph_g27_en_o;
  logic rph_g27_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign rph_g27_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      rph_g27_sel <= 2'b10;
    end else begin
      if (reg_we & rph_g27_sel_addressed) begin
        rph_g27_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g27_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[0][27]
    }),
    .sel_i(rph_g27_sel),
    .out_o(rph_g27_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) rph_g27_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[0][27]
    }),
    .sel_i(rph_g27_sel),
    .out_o(rph_g27_en_o)
  );

  assign rph_g27 = rph_g27_en_o ? rph_g27_o : 1'bz;

  logic [1:0] ah_tmpio0_sel;
  logic ah_tmpio0_o;
  logic ah_tmpio0_en_o;
  logic ah_tmpio0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio0_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio0_sel_addressed) begin
        ah_tmpio0_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][0]
    }),
    .sel_i(ah_tmpio0_sel),
    .out_o(ah_tmpio0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][0]
    }),
    .sel_i(ah_tmpio0_sel),
    .out_o(ah_tmpio0_en_o)
  );

  assign ah_tmpio0 = ah_tmpio0_en_o ? ah_tmpio0_o : 1'bz;

  logic [1:0] ah_tmpio1_sel;
  logic ah_tmpio1_o;
  logic ah_tmpio1_en_o;
  logic ah_tmpio1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio1_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio1_sel_addressed) begin
        ah_tmpio1_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][1]
    }),
    .sel_i(ah_tmpio1_sel),
    .out_o(ah_tmpio1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][1]
    }),
    .sel_i(ah_tmpio1_sel),
    .out_o(ah_tmpio1_en_o)
  );

  assign ah_tmpio1 = ah_tmpio1_en_o ? ah_tmpio1_o : 1'bz;

  logic [1:0] ah_tmpio2_sel;
  logic ah_tmpio2_o;
  logic ah_tmpio2_en_o;
  logic ah_tmpio2_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio2_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio2_sel_addressed) begin
        ah_tmpio2_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][2]
    }),
    .sel_i(ah_tmpio2_sel),
    .out_o(ah_tmpio2_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio2_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][2]
    }),
    .sel_i(ah_tmpio2_sel),
    .out_o(ah_tmpio2_en_o)
  );

  assign ah_tmpio2 = ah_tmpio2_en_o ? ah_tmpio2_o : 1'bz;

  logic [1:0] ah_tmpio3_sel;
  logic ah_tmpio3_o;
  logic ah_tmpio3_en_o;
  logic ah_tmpio3_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio3_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio3_sel_addressed) begin
        ah_tmpio3_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][3]
    }),
    .sel_i(ah_tmpio3_sel),
    .out_o(ah_tmpio3_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio3_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][3]
    }),
    .sel_i(ah_tmpio3_sel),
    .out_o(ah_tmpio3_en_o)
  );

  assign ah_tmpio3 = ah_tmpio3_en_o ? ah_tmpio3_o : 1'bz;

  logic [1:0] ah_tmpio4_sel;
  logic ah_tmpio4_o;
  logic ah_tmpio4_en_o;
  logic ah_tmpio4_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio4_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio4_sel_addressed) begin
        ah_tmpio4_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][4]
    }),
    .sel_i(ah_tmpio4_sel),
    .out_o(ah_tmpio4_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio4_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][4]
    }),
    .sel_i(ah_tmpio4_sel),
    .out_o(ah_tmpio4_en_o)
  );

  assign ah_tmpio4 = ah_tmpio4_en_o ? ah_tmpio4_o : 1'bz;

  logic [1:0] ah_tmpio5_sel;
  logic ah_tmpio5_o;
  logic ah_tmpio5_en_o;
  logic ah_tmpio5_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio5_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio5_sel_addressed) begin
        ah_tmpio5_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][5]
    }),
    .sel_i(ah_tmpio5_sel),
    .out_o(ah_tmpio5_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio5_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][5]
    }),
    .sel_i(ah_tmpio5_sel),
    .out_o(ah_tmpio5_en_o)
  );

  assign ah_tmpio5 = ah_tmpio5_en_o ? ah_tmpio5_o : 1'bz;

  logic [1:0] ah_tmpio6_sel;
  logic ah_tmpio6_o;
  logic ah_tmpio6_en_o;
  logic ah_tmpio6_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio6_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio6_sel_addressed) begin
        ah_tmpio6_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][6]
    }),
    .sel_i(ah_tmpio6_sel),
    .out_o(ah_tmpio6_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio6_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][6]
    }),
    .sel_i(ah_tmpio6_sel),
    .out_o(ah_tmpio6_en_o)
  );

  assign ah_tmpio6 = ah_tmpio6_en_o ? ah_tmpio6_o : 1'bz;

  logic [1:0] ah_tmpio7_sel;
  logic ah_tmpio7_o;
  logic ah_tmpio7_en_o;
  logic ah_tmpio7_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio7_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio7_sel_addressed) begin
        ah_tmpio7_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][7]
    }),
    .sel_i(ah_tmpio7_sel),
    .out_o(ah_tmpio7_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio7_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][7]
    }),
    .sel_i(ah_tmpio7_sel),
    .out_o(ah_tmpio7_en_o)
  );

  assign ah_tmpio7 = ah_tmpio7_en_o ? ah_tmpio7_o : 1'bz;

  logic [1:0] ah_tmpio8_sel;
  logic ah_tmpio8_o;
  logic ah_tmpio8_en_o;
  logic ah_tmpio8_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio8_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio8_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio8_sel_addressed) begin
        ah_tmpio8_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio8_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][8]
    }),
    .sel_i(ah_tmpio8_sel),
    .out_o(ah_tmpio8_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio8_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][8]
    }),
    .sel_i(ah_tmpio8_sel),
    .out_o(ah_tmpio8_en_o)
  );

  assign ah_tmpio8 = ah_tmpio8_en_o ? ah_tmpio8_o : 1'bz;

  logic [1:0] ah_tmpio9_sel;
  logic ah_tmpio9_o;
  logic ah_tmpio9_en_o;
  logic ah_tmpio9_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio9_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio9_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio9_sel_addressed) begin
        ah_tmpio9_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio9_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][9]
    }),
    .sel_i(ah_tmpio9_sel),
    .out_o(ah_tmpio9_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio9_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][9]
    }),
    .sel_i(ah_tmpio9_sel),
    .out_o(ah_tmpio9_en_o)
  );

  assign ah_tmpio9 = ah_tmpio9_en_o ? ah_tmpio9_o : 1'bz;

  logic [1:0] ah_tmpio10_sel;
  logic ah_tmpio10_o;
  logic ah_tmpio10_en_o;
  logic ah_tmpio10_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio10_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio10_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio10_sel_addressed) begin
        ah_tmpio10_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio10_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][10]
    }),
    .sel_i(ah_tmpio10_sel),
    .out_o(ah_tmpio10_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio10_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][10]
    }),
    .sel_i(ah_tmpio10_sel),
    .out_o(ah_tmpio10_en_o)
  );

  assign ah_tmpio10 = ah_tmpio10_en_o ? ah_tmpio10_o : 1'bz;

  logic [2:0] ah_tmpio11_sel;
  logic ah_tmpio11_o;
  logic ah_tmpio11_en_o;
  logic ah_tmpio11_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio11_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio11_sel <= 3'b10;
    end else begin
      if (reg_we & ah_tmpio11_sel_addressed) begin
        ah_tmpio11_sel <= reg_wdata[0+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) ah_tmpio11_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[3],
      gpio_ios_i[1][11]
    }),
    .sel_i(ah_tmpio11_sel),
    .out_o(ah_tmpio11_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) ah_tmpio11_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[1][11]
    }),
    .sel_i(ah_tmpio11_sel),
    .out_o(ah_tmpio11_en_o)
  );

  assign ah_tmpio11 = ah_tmpio11_en_o ? ah_tmpio11_o : 1'bz;

  logic [1:0] ah_tmpio12_sel;
  logic ah_tmpio12_o;
  logic ah_tmpio12_en_o;
  logic ah_tmpio12_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio12_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio12_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio12_sel_addressed) begin
        ah_tmpio12_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio12_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][12]
    }),
    .sel_i(ah_tmpio12_sel),
    .out_o(ah_tmpio12_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio12_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][12]
    }),
    .sel_i(ah_tmpio12_sel),
    .out_o(ah_tmpio12_en_o)
  );

  assign ah_tmpio12 = ah_tmpio12_en_o ? ah_tmpio12_o : 1'bz;

  logic [2:0] ah_tmpio13_sel;
  logic ah_tmpio13_o;
  logic ah_tmpio13_en_o;
  logic ah_tmpio13_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio13_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio13_sel <= 3'b10;
    end else begin
      if (reg_we & ah_tmpio13_sel_addressed) begin
        ah_tmpio13_sel <= reg_wdata[16+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) ah_tmpio13_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[3],
      gpio_ios_i[1][13]
    }),
    .sel_i(ah_tmpio13_sel),
    .out_o(ah_tmpio13_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) ah_tmpio13_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1,
      gpio_ios_en_i[1][13]
    }),
    .sel_i(ah_tmpio13_sel),
    .out_o(ah_tmpio13_en_o)
  );

  assign ah_tmpio13 = ah_tmpio13_en_o ? ah_tmpio13_o : 1'bz;

  logic [1:0] ah_tmpio14_sel;
  logic ah_tmpio14_o;
  logic ah_tmpio14_en_o;
  logic ah_tmpio14_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio14_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio14_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio14_sel_addressed) begin
        ah_tmpio14_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio14_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][14]
    }),
    .sel_i(ah_tmpio14_sel),
    .out_o(ah_tmpio14_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio14_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][14]
    }),
    .sel_i(ah_tmpio14_sel),
    .out_o(ah_tmpio14_en_o)
  );

  assign ah_tmpio14 = ah_tmpio14_en_o ? ah_tmpio14_o : 1'bz;

  logic [1:0] ah_tmpio15_sel;
  logic ah_tmpio15_o;
  logic ah_tmpio15_en_o;
  logic ah_tmpio15_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio15_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio15_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio15_sel_addressed) begin
        ah_tmpio15_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio15_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][15]
    }),
    .sel_i(ah_tmpio15_sel),
    .out_o(ah_tmpio15_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio15_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][15]
    }),
    .sel_i(ah_tmpio15_sel),
    .out_o(ah_tmpio15_en_o)
  );

  assign ah_tmpio15 = ah_tmpio15_en_o ? ah_tmpio15_o : 1'bz;

  logic [1:0] ah_tmpio16_sel;
  logic ah_tmpio16_o;
  logic ah_tmpio16_en_o;
  logic ah_tmpio16_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio16_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio16_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio16_sel_addressed) begin
        ah_tmpio16_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio16_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][16]
    }),
    .sel_i(ah_tmpio16_sel),
    .out_o(ah_tmpio16_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio16_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][16]
    }),
    .sel_i(ah_tmpio16_sel),
    .out_o(ah_tmpio16_en_o)
  );

  assign ah_tmpio16 = ah_tmpio16_en_o ? ah_tmpio16_o : 1'bz;

  logic [1:0] ah_tmpio17_sel;
  logic ah_tmpio17_o;
  logic ah_tmpio17_en_o;
  logic ah_tmpio17_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign ah_tmpio17_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      ah_tmpio17_sel <= 2'b10;
    end else begin
      if (reg_we & ah_tmpio17_sel_addressed) begin
        ah_tmpio17_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio17_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[1][17]
    }),
    .sel_i(ah_tmpio17_sel),
    .out_o(ah_tmpio17_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) ah_tmpio17_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[1][17]
    }),
    .sel_i(ah_tmpio17_sel),
    .out_o(ah_tmpio17_en_o)
  );

  assign ah_tmpio17 = ah_tmpio17_en_o ? ah_tmpio17_o : 1'bz;

  logic [1:0] mb2_sel;
  logic mb2_o;
  logic mb2_en_o;
  logic mb2_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign mb2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      mb2_sel <= 2'b10;
    end else begin
      if (reg_we & mb2_sel_addressed) begin
        mb2_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_sck_i[4]
    }),
    .sel_i(mb2_sel),
    .out_o(mb2_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb2_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(mb2_sel),
    .out_o(mb2_en_o)
  );

  assign mb2 = mb2_en_o ? mb2_o : 1'bz;

  logic [1:0] mb4_sel;
  logic mb4_o;
  logic mb4_en_o;
  logic mb4_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign mb4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      mb4_sel <= 2'b10;
    end else begin
      if (reg_we & mb4_sel_addressed) begin
        mb4_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      spi_tx_i[4]
    }),
    .sel_i(mb4_sel),
    .out_o(mb4_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb4_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(mb4_sel),
    .out_o(mb4_en_o)
  );

  assign mb4 = mb4_en_o ? mb4_o : 1'bz;

  logic [1:0] mb5_sel;
  logic mb5_o;
  logic mb5_en_o;
  logic mb5_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign mb5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      mb5_sel <= 2'b10;
    end else begin
      if (reg_we & mb5_sel_addressed) begin
        mb5_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_sda_i[1]
    }),
    .sel_i(mb5_sel),
    .out_o(mb5_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb5_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_sda_en_i[1]
    }),
    .sel_i(mb5_sel),
    .out_o(mb5_en_o)
  );

  assign mb5 = mb5_en_o ? mb5_o : 1'bz;

  logic [1:0] mb6_sel;
  logic mb6_o;
  logic mb6_en_o;
  logic mb6_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign mb6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      mb6_sel <= 2'b10;
    end else begin
      if (reg_we & mb6_sel_addressed) begin
        mb6_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      i2c_scl_i[1]
    }),
    .sel_i(mb6_sel),
    .out_o(mb6_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb6_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      i2c_scl_en_i[1]
    }),
    .sel_i(mb6_sel),
    .out_o(mb6_en_o)
  );

  assign mb6 = mb6_en_o ? mb6_o : 1'bz;

  logic [1:0] mb7_sel;
  logic mb7_o;
  logic mb7_en_o;
  logic mb7_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign mb7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      mb7_sel <= 2'b10;
    end else begin
      if (reg_we & mb7_sel_addressed) begin
        mb7_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      uart_tx_i[3]
    }),
    .sel_i(mb7_sel),
    .out_o(mb7_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) mb7_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      '1
    }),
    .sel_i(mb7_sel),
    .out_o(mb7_en_o)
  );

  assign mb7 = mb7_en_o ? mb7_o : 1'bz;

  logic [1:0] pmod0_0_sel;
  logic pmod0_0_o;
  logic pmod0_0_en_o;
  logic pmod0_0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_0_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_0_sel_addressed) begin
        pmod0_0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][0]
    }),
    .sel_i(pmod0_0_sel),
    .out_o(pmod0_0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][0]
    }),
    .sel_i(pmod0_0_sel),
    .out_o(pmod0_0_en_o)
  );

  assign pmod0[0] = pmod0_0_en_o ? pmod0_0_o : 1'bz;

  logic [1:0] pmod0_1_sel;
  logic pmod0_1_o;
  logic pmod0_1_en_o;
  logic pmod0_1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_1_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_1_sel_addressed) begin
        pmod0_1_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][1]
    }),
    .sel_i(pmod0_1_sel),
    .out_o(pmod0_1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][1]
    }),
    .sel_i(pmod0_1_sel),
    .out_o(pmod0_1_en_o)
  );

  assign pmod0[1] = pmod0_1_en_o ? pmod0_1_o : 1'bz;

  logic [1:0] pmod0_2_sel;
  logic pmod0_2_o;
  logic pmod0_2_en_o;
  logic pmod0_2_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_2_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_2_sel_addressed) begin
        pmod0_2_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][2]
    }),
    .sel_i(pmod0_2_sel),
    .out_o(pmod0_2_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_2_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][2]
    }),
    .sel_i(pmod0_2_sel),
    .out_o(pmod0_2_en_o)
  );

  assign pmod0[2] = pmod0_2_en_o ? pmod0_2_o : 1'bz;

  logic [1:0] pmod0_3_sel;
  logic pmod0_3_o;
  logic pmod0_3_en_o;
  logic pmod0_3_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_3_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_3_sel_addressed) begin
        pmod0_3_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][3]
    }),
    .sel_i(pmod0_3_sel),
    .out_o(pmod0_3_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_3_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][3]
    }),
    .sel_i(pmod0_3_sel),
    .out_o(pmod0_3_en_o)
  );

  assign pmod0[3] = pmod0_3_en_o ? pmod0_3_o : 1'bz;

  logic [1:0] pmod0_4_sel;
  logic pmod0_4_o;
  logic pmod0_4_en_o;
  logic pmod0_4_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_4_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_4_sel_addressed) begin
        pmod0_4_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][4]
    }),
    .sel_i(pmod0_4_sel),
    .out_o(pmod0_4_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_4_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][4]
    }),
    .sel_i(pmod0_4_sel),
    .out_o(pmod0_4_en_o)
  );

  assign pmod0[4] = pmod0_4_en_o ? pmod0_4_o : 1'bz;

  logic [1:0] pmod0_5_sel;
  logic pmod0_5_o;
  logic pmod0_5_en_o;
  logic pmod0_5_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_5_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_5_sel_addressed) begin
        pmod0_5_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][5]
    }),
    .sel_i(pmod0_5_sel),
    .out_o(pmod0_5_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_5_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][5]
    }),
    .sel_i(pmod0_5_sel),
    .out_o(pmod0_5_en_o)
  );

  assign pmod0[5] = pmod0_5_en_o ? pmod0_5_o : 1'bz;

  logic [1:0] pmod0_6_sel;
  logic pmod0_6_o;
  logic pmod0_6_en_o;
  logic pmod0_6_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_6_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_6_sel_addressed) begin
        pmod0_6_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][6]
    }),
    .sel_i(pmod0_6_sel),
    .out_o(pmod0_6_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_6_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][6]
    }),
    .sel_i(pmod0_6_sel),
    .out_o(pmod0_6_en_o)
  );

  assign pmod0[6] = pmod0_6_en_o ? pmod0_6_o : 1'bz;

  logic [1:0] pmod0_7_sel;
  logic pmod0_7_o;
  logic pmod0_7_en_o;
  logic pmod0_7_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod0_7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod0_7_sel <= 2'b10;
    end else begin
      if (reg_we & pmod0_7_sel_addressed) begin
        pmod0_7_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][7]
    }),
    .sel_i(pmod0_7_sel),
    .out_o(pmod0_7_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod0_7_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][7]
    }),
    .sel_i(pmod0_7_sel),
    .out_o(pmod0_7_en_o)
  );

  assign pmod0[7] = pmod0_7_en_o ? pmod0_7_o : 1'bz;

  logic [1:0] pmod1_0_sel;
  logic pmod1_0_o;
  logic pmod1_0_en_o;
  logic pmod1_0_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_0_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_0_sel_addressed) begin
        pmod1_0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][8]
    }),
    .sel_i(pmod1_0_sel),
    .out_o(pmod1_0_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_0_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][8]
    }),
    .sel_i(pmod1_0_sel),
    .out_o(pmod1_0_en_o)
  );

  assign pmod1[0] = pmod1_0_en_o ? pmod1_0_o : 1'bz;

  logic [1:0] pmod1_1_sel;
  logic pmod1_1_o;
  logic pmod1_1_en_o;
  logic pmod1_1_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_1_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_1_sel_addressed) begin
        pmod1_1_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][9]
    }),
    .sel_i(pmod1_1_sel),
    .out_o(pmod1_1_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_1_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][9]
    }),
    .sel_i(pmod1_1_sel),
    .out_o(pmod1_1_en_o)
  );

  assign pmod1[1] = pmod1_1_en_o ? pmod1_1_o : 1'bz;

  logic [1:0] pmod1_2_sel;
  logic pmod1_2_o;
  logic pmod1_2_en_o;
  logic pmod1_2_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_2_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_2_sel_addressed) begin
        pmod1_2_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][10]
    }),
    .sel_i(pmod1_2_sel),
    .out_o(pmod1_2_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_2_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][10]
    }),
    .sel_i(pmod1_2_sel),
    .out_o(pmod1_2_en_o)
  );

  assign pmod1[2] = pmod1_2_en_o ? pmod1_2_o : 1'bz;

  logic [1:0] pmod1_3_sel;
  logic pmod1_3_o;
  logic pmod1_3_en_o;
  logic pmod1_3_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_3_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_3_sel_addressed) begin
        pmod1_3_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][11]
    }),
    .sel_i(pmod1_3_sel),
    .out_o(pmod1_3_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_3_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][11]
    }),
    .sel_i(pmod1_3_sel),
    .out_o(pmod1_3_en_o)
  );

  assign pmod1[3] = pmod1_3_en_o ? pmod1_3_o : 1'bz;

  logic [1:0] pmod1_4_sel;
  logic pmod1_4_o;
  logic pmod1_4_en_o;
  logic pmod1_4_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_4_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_4_sel_addressed) begin
        pmod1_4_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][12]
    }),
    .sel_i(pmod1_4_sel),
    .out_o(pmod1_4_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_4_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][12]
    }),
    .sel_i(pmod1_4_sel),
    .out_o(pmod1_4_en_o)
  );

  assign pmod1[4] = pmod1_4_en_o ? pmod1_4_o : 1'bz;

  logic [1:0] pmod1_5_sel;
  logic pmod1_5_o;
  logic pmod1_5_en_o;
  logic pmod1_5_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_5_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_5_sel_addressed) begin
        pmod1_5_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][13]
    }),
    .sel_i(pmod1_5_sel),
    .out_o(pmod1_5_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_5_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][13]
    }),
    .sel_i(pmod1_5_sel),
    .out_o(pmod1_5_en_o)
  );

  assign pmod1[5] = pmod1_5_en_o ? pmod1_5_o : 1'bz;

  logic [1:0] pmod1_6_sel;
  logic pmod1_6_o;
  logic pmod1_6_en_o;
  logic pmod1_6_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_6_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_6_sel_addressed) begin
        pmod1_6_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][14]
    }),
    .sel_i(pmod1_6_sel),
    .out_o(pmod1_6_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_6_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][14]
    }),
    .sel_i(pmod1_6_sel),
    .out_o(pmod1_6_en_o)
  );

  assign pmod1[6] = pmod1_6_en_o ? pmod1_6_o : 1'bz;

  logic [1:0] pmod1_7_sel;
  logic pmod1_7_o;
  logic pmod1_7_en_o;
  logic pmod1_7_sel_addressed;

  // Register addresses of 0x000 to 0x7ff are pin selectors, which are packed with 4 per 32-bit word.
  assign pmod1_7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b0 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      pmod1_7_sel <= 2'b10;
    end else begin
      if (reg_we & pmod1_7_sel_addressed) begin
        pmod1_7_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0, // This is set to Z later when output enable is low.
      gpio_ios_i[2][15]
    }),
    .sel_i(pmod1_7_sel),
    .out_o(pmod1_7_o)
  );

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) pmod1_7_enable_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      gpio_ios_en_i[2][15]
    }),
    .sel_i(pmod1_7_sel),
    .out_o(pmod1_7_en_o)
  );

  assign pmod1[7] = pmod1_7_en_o ? pmod1_7_o : 1'bz;

  // Inputs - Physical pin inputs are muxed to particular block IO

  logic [1:0] uart_rx_0_sel;
  logic uart_rx_0_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign uart_rx_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      uart_rx_0_sel <= 2'b10;
    end else begin
      if (reg_we & uart_rx_0_sel_addressed) begin
        uart_rx_0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) uart_rx_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b1,
      ser0_rx
    }),
    .sel_i(uart_rx_0_sel),
    .out_o(uart_rx_o[0])
  );

  logic [1:0] uart_rx_1_sel;
  logic uart_rx_1_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign uart_rx_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      uart_rx_1_sel <= 2'b10;
    end else begin
      if (reg_we & uart_rx_1_sel_addressed) begin
        uart_rx_1_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) uart_rx_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b1,
      ser1_rx
    }),
    .sel_i(uart_rx_1_sel),
    .out_o(uart_rx_o[1])
  );

  logic [1:0] uart_rx_2_sel;
  logic uart_rx_2_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign uart_rx_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      uart_rx_2_sel <= 2'b10;
    end else begin
      if (reg_we & uart_rx_2_sel_addressed) begin
        uart_rx_2_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) uart_rx_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b1,
      rph_rxd0
    }),
    .sel_i(uart_rx_2_sel),
    .out_o(uart_rx_o[2])
  );

  logic [1:0] uart_rx_3_sel;
  logic uart_rx_3_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign uart_rx_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 0 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      uart_rx_3_sel <= 2'b10;
    end else begin
      if (reg_we & uart_rx_3_sel_addressed) begin
        uart_rx_3_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) uart_rx_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b1,
      mb8
    }),
    .sel_i(uart_rx_3_sel),
    .out_o(uart_rx_o[3])
  );

  logic [1:0] uart_rx_4_sel;
  logic uart_rx_4_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign uart_rx_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      uart_rx_4_sel <= 2'b10;
    end else begin
      if (reg_we & uart_rx_4_sel_addressed) begin
        uart_rx_4_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) uart_rx_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b1,
      rs232_rx
    }),
    .sel_i(uart_rx_4_sel),
    .out_o(uart_rx_o[4])
  );

  logic [1:0] spi_rx_0_sel;
  logic spi_rx_0_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign spi_rx_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      spi_rx_0_sel <= 2'b10;
    end else begin
      if (reg_we & spi_rx_0_sel_addressed) begin
        spi_rx_0_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) spi_rx_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      appspi_d1
    }),
    .sel_i(spi_rx_0_sel),
    .out_o(spi_rx_o[0])
  );

  logic [1:0] spi_rx_1_sel;
  logic spi_rx_1_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign spi_rx_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      spi_rx_1_sel <= 2'b10;
    end else begin
      if (reg_we & spi_rx_1_sel_addressed) begin
        spi_rx_1_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) spi_rx_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(spi_rx_1_sel),
    .out_o(spi_rx_o[1])
  );

  logic [1:0] spi_rx_2_sel;
  logic spi_rx_2_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign spi_rx_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 4 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      spi_rx_2_sel <= 2'b10;
    end else begin
      if (reg_we & spi_rx_2_sel_addressed) begin
        spi_rx_2_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) spi_rx_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ethmac_cipo
    }),
    .sel_i(spi_rx_2_sel),
    .out_o(spi_rx_o[2])
  );

  logic [2:0] spi_rx_3_sel;
  logic spi_rx_3_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign spi_rx_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      spi_rx_3_sel <= 3'b10;
    end else begin
      if (reg_we & spi_rx_3_sel_addressed) begin
        spi_rx_3_sel <= reg_wdata[0+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) spi_rx_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g9_cipo,
      ah_tmpio12
    }),
    .sel_i(spi_rx_3_sel),
    .out_o(spi_rx_o[3])
  );

  logic [2:0] spi_rx_4_sel;
  logic spi_rx_4_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign spi_rx_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      spi_rx_4_sel <= 3'b10;
    end else begin
      if (reg_we & spi_rx_4_sel_addressed) begin
        spi_rx_4_sel <= reg_wdata[8+:3];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(3)
  ) spi_rx_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g19_cipo,
      mb3
    }),
    .sel_i(spi_rx_4_sel),
    .out_o(spi_rx_o[4])
  );

  logic [1:0] gpio_ios_0_0_sel;
  logic gpio_ios_0_0_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_0_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_0_sel_addressed) begin
        gpio_ios_0_0_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g0
    }),
    .sel_i(gpio_ios_0_0_sel),
    .out_o(gpio_ios_o[0][0])
  );

  logic [1:0] gpio_ios_1_0_sel;
  logic gpio_ios_1_0_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 8 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_0_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_0_sel_addressed) begin
        gpio_ios_1_0_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio0
    }),
    .sel_i(gpio_ios_1_0_sel),
    .out_o(gpio_ios_o[1][0])
  );

  logic [1:0] gpio_ios_2_0_sel;
  logic gpio_ios_2_0_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_0_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_0_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_0_sel_addressed) begin
        gpio_ios_2_0_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_0_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[0]
    }),
    .sel_i(gpio_ios_2_0_sel),
    .out_o(gpio_ios_o[2][0])
  );

  logic [1:0] gpio_ios_0_1_sel;
  logic gpio_ios_0_1_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_1_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_1_sel_addressed) begin
        gpio_ios_0_1_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g1
    }),
    .sel_i(gpio_ios_0_1_sel),
    .out_o(gpio_ios_o[0][1])
  );

  logic [1:0] gpio_ios_1_1_sel;
  logic gpio_ios_1_1_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_1_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_1_sel_addressed) begin
        gpio_ios_1_1_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio1
    }),
    .sel_i(gpio_ios_1_1_sel),
    .out_o(gpio_ios_o[1][1])
  );

  logic [1:0] gpio_ios_2_1_sel;
  logic gpio_ios_2_1_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_1_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 12 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_1_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_1_sel_addressed) begin
        gpio_ios_2_1_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_1_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[1]
    }),
    .sel_i(gpio_ios_2_1_sel),
    .out_o(gpio_ios_o[2][1])
  );

  logic [1:0] gpio_ios_0_2_sel;
  logic gpio_ios_0_2_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_2_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_2_sel_addressed) begin
        gpio_ios_0_2_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g2_sda
    }),
    .sel_i(gpio_ios_0_2_sel),
    .out_o(gpio_ios_o[0][2])
  );

  logic [1:0] gpio_ios_1_2_sel;
  logic gpio_ios_1_2_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_2_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_2_sel_addressed) begin
        gpio_ios_1_2_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio2
    }),
    .sel_i(gpio_ios_1_2_sel),
    .out_o(gpio_ios_o[1][2])
  );

  logic [1:0] gpio_ios_2_2_sel;
  logic gpio_ios_2_2_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_2_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_2_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_2_sel_addressed) begin
        gpio_ios_2_2_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_2_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[2]
    }),
    .sel_i(gpio_ios_2_2_sel),
    .out_o(gpio_ios_o[2][2])
  );

  logic [1:0] gpio_ios_0_3_sel;
  logic gpio_ios_0_3_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 16 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_3_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_3_sel_addressed) begin
        gpio_ios_0_3_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g3_scl
    }),
    .sel_i(gpio_ios_0_3_sel),
    .out_o(gpio_ios_o[0][3])
  );

  logic [1:0] gpio_ios_1_3_sel;
  logic gpio_ios_1_3_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_3_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_3_sel_addressed) begin
        gpio_ios_1_3_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio3
    }),
    .sel_i(gpio_ios_1_3_sel),
    .out_o(gpio_ios_o[1][3])
  );

  logic [1:0] gpio_ios_2_3_sel;
  logic gpio_ios_2_3_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_3_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_3_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_3_sel_addressed) begin
        gpio_ios_2_3_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_3_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[3]
    }),
    .sel_i(gpio_ios_2_3_sel),
    .out_o(gpio_ios_o[2][3])
  );

  logic [1:0] gpio_ios_0_4_sel;
  logic gpio_ios_0_4_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_4_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_4_sel_addressed) begin
        gpio_ios_0_4_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g4
    }),
    .sel_i(gpio_ios_0_4_sel),
    .out_o(gpio_ios_o[0][4])
  );

  logic [1:0] gpio_ios_1_4_sel;
  logic gpio_ios_1_4_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 20 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_4_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_4_sel_addressed) begin
        gpio_ios_1_4_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio4
    }),
    .sel_i(gpio_ios_1_4_sel),
    .out_o(gpio_ios_o[1][4])
  );

  logic [1:0] gpio_ios_2_4_sel;
  logic gpio_ios_2_4_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_4_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_4_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_4_sel_addressed) begin
        gpio_ios_2_4_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_4_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[4]
    }),
    .sel_i(gpio_ios_2_4_sel),
    .out_o(gpio_ios_o[2][4])
  );

  logic [1:0] gpio_ios_0_5_sel;
  logic gpio_ios_0_5_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_5_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_5_sel_addressed) begin
        gpio_ios_0_5_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g5
    }),
    .sel_i(gpio_ios_0_5_sel),
    .out_o(gpio_ios_o[0][5])
  );

  logic [1:0] gpio_ios_1_5_sel;
  logic gpio_ios_1_5_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_5_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_5_sel_addressed) begin
        gpio_ios_1_5_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio5
    }),
    .sel_i(gpio_ios_1_5_sel),
    .out_o(gpio_ios_o[1][5])
  );

  logic [1:0] gpio_ios_2_5_sel;
  logic gpio_ios_2_5_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_5_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 24 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_5_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_5_sel_addressed) begin
        gpio_ios_2_5_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_5_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[5]
    }),
    .sel_i(gpio_ios_2_5_sel),
    .out_o(gpio_ios_o[2][5])
  );

  logic [1:0] gpio_ios_0_6_sel;
  logic gpio_ios_0_6_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_6_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_6_sel_addressed) begin
        gpio_ios_0_6_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g6
    }),
    .sel_i(gpio_ios_0_6_sel),
    .out_o(gpio_ios_o[0][6])
  );

  logic [1:0] gpio_ios_1_6_sel;
  logic gpio_ios_1_6_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_6_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_6_sel_addressed) begin
        gpio_ios_1_6_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio6
    }),
    .sel_i(gpio_ios_1_6_sel),
    .out_o(gpio_ios_o[1][6])
  );

  logic [1:0] gpio_ios_2_6_sel;
  logic gpio_ios_2_6_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_6_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_6_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_6_sel_addressed) begin
        gpio_ios_2_6_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_6_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[6]
    }),
    .sel_i(gpio_ios_2_6_sel),
    .out_o(gpio_ios_o[2][6])
  );

  logic [1:0] gpio_ios_0_7_sel;
  logic gpio_ios_0_7_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 28 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_7_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_7_sel_addressed) begin
        gpio_ios_0_7_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g7_ce1
    }),
    .sel_i(gpio_ios_0_7_sel),
    .out_o(gpio_ios_o[0][7])
  );

  logic [1:0] gpio_ios_1_7_sel;
  logic gpio_ios_1_7_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_7_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_7_sel_addressed) begin
        gpio_ios_1_7_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio7
    }),
    .sel_i(gpio_ios_1_7_sel),
    .out_o(gpio_ios_o[1][7])
  );

  logic [1:0] gpio_ios_2_7_sel;
  logic gpio_ios_2_7_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_7_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_7_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_7_sel_addressed) begin
        gpio_ios_2_7_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_7_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod0[7]
    }),
    .sel_i(gpio_ios_2_7_sel),
    .out_o(gpio_ios_o[2][7])
  );

  logic [1:0] gpio_ios_0_8_sel;
  logic gpio_ios_0_8_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_8_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_8_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_8_sel_addressed) begin
        gpio_ios_0_8_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_8_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g8_ce0
    }),
    .sel_i(gpio_ios_0_8_sel),
    .out_o(gpio_ios_o[0][8])
  );

  logic [1:0] gpio_ios_1_8_sel;
  logic gpio_ios_1_8_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_8_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 32 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_8_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_8_sel_addressed) begin
        gpio_ios_1_8_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_8_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio8
    }),
    .sel_i(gpio_ios_1_8_sel),
    .out_o(gpio_ios_o[1][8])
  );

  logic [1:0] gpio_ios_2_8_sel;
  logic gpio_ios_2_8_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_8_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_8_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_8_sel_addressed) begin
        gpio_ios_2_8_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_8_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[0]
    }),
    .sel_i(gpio_ios_2_8_sel),
    .out_o(gpio_ios_o[2][8])
  );

  logic [1:0] gpio_ios_0_9_sel;
  logic gpio_ios_0_9_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_9_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_9_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_9_sel_addressed) begin
        gpio_ios_0_9_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_9_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g9_cipo
    }),
    .sel_i(gpio_ios_0_9_sel),
    .out_o(gpio_ios_o[0][9])
  );

  logic [1:0] gpio_ios_1_9_sel;
  logic gpio_ios_1_9_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_9_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_9_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_9_sel_addressed) begin
        gpio_ios_1_9_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_9_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio9
    }),
    .sel_i(gpio_ios_1_9_sel),
    .out_o(gpio_ios_o[1][9])
  );

  logic [1:0] gpio_ios_2_9_sel;
  logic gpio_ios_2_9_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_9_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 36 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_9_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_9_sel_addressed) begin
        gpio_ios_2_9_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_9_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[1]
    }),
    .sel_i(gpio_ios_2_9_sel),
    .out_o(gpio_ios_o[2][9])
  );

  logic [1:0] gpio_ios_0_10_sel;
  logic gpio_ios_0_10_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_10_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_10_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_10_sel_addressed) begin
        gpio_ios_0_10_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_10_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g10_copi
    }),
    .sel_i(gpio_ios_0_10_sel),
    .out_o(gpio_ios_o[0][10])
  );

  logic [1:0] gpio_ios_1_10_sel;
  logic gpio_ios_1_10_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_10_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_10_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_10_sel_addressed) begin
        gpio_ios_1_10_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_10_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio10
    }),
    .sel_i(gpio_ios_1_10_sel),
    .out_o(gpio_ios_o[1][10])
  );

  logic [1:0] gpio_ios_2_10_sel;
  logic gpio_ios_2_10_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_10_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_10_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_10_sel_addressed) begin
        gpio_ios_2_10_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_10_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[2]
    }),
    .sel_i(gpio_ios_2_10_sel),
    .out_o(gpio_ios_o[2][10])
  );

  logic [1:0] gpio_ios_0_11_sel;
  logic gpio_ios_0_11_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_11_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 40 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_11_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_11_sel_addressed) begin
        gpio_ios_0_11_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_11_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g11_sclk
    }),
    .sel_i(gpio_ios_0_11_sel),
    .out_o(gpio_ios_o[0][11])
  );

  logic [1:0] gpio_ios_1_11_sel;
  logic gpio_ios_1_11_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_11_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_11_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_11_sel_addressed) begin
        gpio_ios_1_11_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_11_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio11
    }),
    .sel_i(gpio_ios_1_11_sel),
    .out_o(gpio_ios_o[1][11])
  );

  logic [1:0] gpio_ios_2_11_sel;
  logic gpio_ios_2_11_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_11_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_11_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_11_sel_addressed) begin
        gpio_ios_2_11_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_11_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[3]
    }),
    .sel_i(gpio_ios_2_11_sel),
    .out_o(gpio_ios_o[2][11])
  );

  logic [1:0] gpio_ios_0_12_sel;
  logic gpio_ios_0_12_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_12_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_12_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_12_sel_addressed) begin
        gpio_ios_0_12_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_12_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g12
    }),
    .sel_i(gpio_ios_0_12_sel),
    .out_o(gpio_ios_o[0][12])
  );

  logic [1:0] gpio_ios_1_12_sel;
  logic gpio_ios_1_12_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_12_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 44 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_12_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_12_sel_addressed) begin
        gpio_ios_1_12_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_12_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio12
    }),
    .sel_i(gpio_ios_1_12_sel),
    .out_o(gpio_ios_o[1][12])
  );

  logic [1:0] gpio_ios_2_12_sel;
  logic gpio_ios_2_12_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_12_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_12_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_12_sel_addressed) begin
        gpio_ios_2_12_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_12_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[4]
    }),
    .sel_i(gpio_ios_2_12_sel),
    .out_o(gpio_ios_o[2][12])
  );

  logic [1:0] gpio_ios_0_13_sel;
  logic gpio_ios_0_13_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_13_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_13_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_13_sel_addressed) begin
        gpio_ios_0_13_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_13_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g13
    }),
    .sel_i(gpio_ios_0_13_sel),
    .out_o(gpio_ios_o[0][13])
  );

  logic [1:0] gpio_ios_1_13_sel;
  logic gpio_ios_1_13_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_13_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_13_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_13_sel_addressed) begin
        gpio_ios_1_13_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_13_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio13
    }),
    .sel_i(gpio_ios_1_13_sel),
    .out_o(gpio_ios_o[1][13])
  );

  logic [1:0] gpio_ios_2_13_sel;
  logic gpio_ios_2_13_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_13_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 48 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_13_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_13_sel_addressed) begin
        gpio_ios_2_13_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_13_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[5]
    }),
    .sel_i(gpio_ios_2_13_sel),
    .out_o(gpio_ios_o[2][13])
  );

  logic [1:0] gpio_ios_0_14_sel;
  logic gpio_ios_0_14_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_14_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_14_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_14_sel_addressed) begin
        gpio_ios_0_14_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_14_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_txd0
    }),
    .sel_i(gpio_ios_0_14_sel),
    .out_o(gpio_ios_o[0][14])
  );

  logic [1:0] gpio_ios_1_14_sel;
  logic gpio_ios_1_14_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_14_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_14_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_14_sel_addressed) begin
        gpio_ios_1_14_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_14_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio14
    }),
    .sel_i(gpio_ios_1_14_sel),
    .out_o(gpio_ios_o[1][14])
  );

  logic [1:0] gpio_ios_2_14_sel;
  logic gpio_ios_2_14_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_14_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_14_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_14_sel_addressed) begin
        gpio_ios_2_14_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_14_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[6]
    }),
    .sel_i(gpio_ios_2_14_sel),
    .out_o(gpio_ios_o[2][14])
  );

  logic [1:0] gpio_ios_0_15_sel;
  logic gpio_ios_0_15_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_15_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 52 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_15_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_15_sel_addressed) begin
        gpio_ios_0_15_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_15_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_rxd0
    }),
    .sel_i(gpio_ios_0_15_sel),
    .out_o(gpio_ios_o[0][15])
  );

  logic [1:0] gpio_ios_1_15_sel;
  logic gpio_ios_1_15_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_15_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_15_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_15_sel_addressed) begin
        gpio_ios_1_15_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_15_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio15
    }),
    .sel_i(gpio_ios_1_15_sel),
    .out_o(gpio_ios_o[1][15])
  );

  logic [1:0] gpio_ios_2_15_sel;
  logic gpio_ios_2_15_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_15_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_15_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_15_sel_addressed) begin
        gpio_ios_2_15_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_15_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      pmod1[7]
    }),
    .sel_i(gpio_ios_2_15_sel),
    .out_o(gpio_ios_o[2][15])
  );

  logic [1:0] gpio_ios_0_16_sel;
  logic gpio_ios_0_16_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_16_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_16_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_16_sel_addressed) begin
        gpio_ios_0_16_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_16_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g16_ce2
    }),
    .sel_i(gpio_ios_0_16_sel),
    .out_o(gpio_ios_o[0][16])
  );

  logic [1:0] gpio_ios_1_16_sel;
  logic gpio_ios_1_16_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_16_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 56 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_16_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_16_sel_addressed) begin
        gpio_ios_1_16_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_16_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio16
    }),
    .sel_i(gpio_ios_1_16_sel),
    .out_o(gpio_ios_o[1][16])
  );

  logic [1:0] gpio_ios_2_16_sel;
  logic gpio_ios_2_16_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_16_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_16_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_16_sel_addressed) begin
        gpio_ios_2_16_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_16_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_16_sel),
    .out_o(gpio_ios_o[2][16])
  );

  logic [1:0] gpio_ios_0_17_sel;
  logic gpio_ios_0_17_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_17_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_17_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_17_sel_addressed) begin
        gpio_ios_0_17_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_17_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g17
    }),
    .sel_i(gpio_ios_0_17_sel),
    .out_o(gpio_ios_o[0][17])
  );

  logic [1:0] gpio_ios_1_17_sel;
  logic gpio_ios_1_17_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_17_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_17_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_17_sel_addressed) begin
        gpio_ios_1_17_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_17_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      ah_tmpio17
    }),
    .sel_i(gpio_ios_1_17_sel),
    .out_o(gpio_ios_o[1][17])
  );

  logic [1:0] gpio_ios_2_17_sel;
  logic gpio_ios_2_17_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_17_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 60 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_17_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_17_sel_addressed) begin
        gpio_ios_2_17_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_17_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_17_sel),
    .out_o(gpio_ios_o[2][17])
  );

  logic [1:0] gpio_ios_0_18_sel;
  logic gpio_ios_0_18_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_18_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_18_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_18_sel_addressed) begin
        gpio_ios_0_18_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_18_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g18
    }),
    .sel_i(gpio_ios_0_18_sel),
    .out_o(gpio_ios_o[0][18])
  );

  logic [1:0] gpio_ios_1_18_sel;
  logic gpio_ios_1_18_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_18_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_18_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_18_sel_addressed) begin
        gpio_ios_1_18_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_18_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_18_sel),
    .out_o(gpio_ios_o[1][18])
  );

  logic [1:0] gpio_ios_2_18_sel;
  logic gpio_ios_2_18_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_18_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_18_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_18_sel_addressed) begin
        gpio_ios_2_18_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_18_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_18_sel),
    .out_o(gpio_ios_o[2][18])
  );

  logic [1:0] gpio_ios_0_19_sel;
  logic gpio_ios_0_19_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_19_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 64 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_19_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_19_sel_addressed) begin
        gpio_ios_0_19_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_19_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g19_cipo
    }),
    .sel_i(gpio_ios_0_19_sel),
    .out_o(gpio_ios_o[0][19])
  );

  logic [1:0] gpio_ios_1_19_sel;
  logic gpio_ios_1_19_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_19_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_19_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_19_sel_addressed) begin
        gpio_ios_1_19_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_19_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_19_sel),
    .out_o(gpio_ios_o[1][19])
  );

  logic [1:0] gpio_ios_2_19_sel;
  logic gpio_ios_2_19_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_19_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_19_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_19_sel_addressed) begin
        gpio_ios_2_19_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_19_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_19_sel),
    .out_o(gpio_ios_o[2][19])
  );

  logic [1:0] gpio_ios_0_20_sel;
  logic gpio_ios_0_20_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_20_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_20_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_20_sel_addressed) begin
        gpio_ios_0_20_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_20_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g20_copi
    }),
    .sel_i(gpio_ios_0_20_sel),
    .out_o(gpio_ios_o[0][20])
  );

  logic [1:0] gpio_ios_1_20_sel;
  logic gpio_ios_1_20_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_20_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 68 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_20_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_20_sel_addressed) begin
        gpio_ios_1_20_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_20_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_20_sel),
    .out_o(gpio_ios_o[1][20])
  );

  logic [1:0] gpio_ios_2_20_sel;
  logic gpio_ios_2_20_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_20_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_20_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_20_sel_addressed) begin
        gpio_ios_2_20_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_20_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_20_sel),
    .out_o(gpio_ios_o[2][20])
  );

  logic [1:0] gpio_ios_0_21_sel;
  logic gpio_ios_0_21_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_21_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_21_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_21_sel_addressed) begin
        gpio_ios_0_21_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_21_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g21_sclk
    }),
    .sel_i(gpio_ios_0_21_sel),
    .out_o(gpio_ios_o[0][21])
  );

  logic [1:0] gpio_ios_1_21_sel;
  logic gpio_ios_1_21_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_21_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_21_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_21_sel_addressed) begin
        gpio_ios_1_21_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_21_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_21_sel),
    .out_o(gpio_ios_o[1][21])
  );

  logic [1:0] gpio_ios_2_21_sel;
  logic gpio_ios_2_21_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_21_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 72 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_21_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_21_sel_addressed) begin
        gpio_ios_2_21_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_21_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_21_sel),
    .out_o(gpio_ios_o[2][21])
  );

  logic [1:0] gpio_ios_0_22_sel;
  logic gpio_ios_0_22_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_22_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_22_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_22_sel_addressed) begin
        gpio_ios_0_22_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_22_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g22
    }),
    .sel_i(gpio_ios_0_22_sel),
    .out_o(gpio_ios_o[0][22])
  );

  logic [1:0] gpio_ios_1_22_sel;
  logic gpio_ios_1_22_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_22_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_22_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_22_sel_addressed) begin
        gpio_ios_1_22_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_22_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_22_sel),
    .out_o(gpio_ios_o[1][22])
  );

  logic [1:0] gpio_ios_2_22_sel;
  logic gpio_ios_2_22_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_22_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_22_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_22_sel_addressed) begin
        gpio_ios_2_22_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_22_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_22_sel),
    .out_o(gpio_ios_o[2][22])
  );

  logic [1:0] gpio_ios_0_23_sel;
  logic gpio_ios_0_23_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_23_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 76 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_23_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_23_sel_addressed) begin
        gpio_ios_0_23_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_23_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g23
    }),
    .sel_i(gpio_ios_0_23_sel),
    .out_o(gpio_ios_o[0][23])
  );

  logic [1:0] gpio_ios_1_23_sel;
  logic gpio_ios_1_23_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_23_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 80 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_23_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_23_sel_addressed) begin
        gpio_ios_1_23_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_23_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_23_sel),
    .out_o(gpio_ios_o[1][23])
  );

  logic [1:0] gpio_ios_2_23_sel;
  logic gpio_ios_2_23_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_23_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 80 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_23_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_23_sel_addressed) begin
        gpio_ios_2_23_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_23_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_23_sel),
    .out_o(gpio_ios_o[2][23])
  );

  logic [1:0] gpio_ios_0_24_sel;
  logic gpio_ios_0_24_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_24_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 80 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_24_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_24_sel_addressed) begin
        gpio_ios_0_24_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_24_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g24
    }),
    .sel_i(gpio_ios_0_24_sel),
    .out_o(gpio_ios_o[0][24])
  );

  logic [1:0] gpio_ios_1_24_sel;
  logic gpio_ios_1_24_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_24_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 80 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_24_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_24_sel_addressed) begin
        gpio_ios_1_24_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_24_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_24_sel),
    .out_o(gpio_ios_o[1][24])
  );

  logic [1:0] gpio_ios_2_24_sel;
  logic gpio_ios_2_24_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_24_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 84 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_24_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_24_sel_addressed) begin
        gpio_ios_2_24_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_24_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_24_sel),
    .out_o(gpio_ios_o[2][24])
  );

  logic [1:0] gpio_ios_0_25_sel;
  logic gpio_ios_0_25_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_25_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 84 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_25_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_25_sel_addressed) begin
        gpio_ios_0_25_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_25_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g25
    }),
    .sel_i(gpio_ios_0_25_sel),
    .out_o(gpio_ios_o[0][25])
  );

  logic [1:0] gpio_ios_1_25_sel;
  logic gpio_ios_1_25_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_25_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 84 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_25_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_25_sel_addressed) begin
        gpio_ios_1_25_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_25_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_25_sel),
    .out_o(gpio_ios_o[1][25])
  );

  logic [1:0] gpio_ios_2_25_sel;
  logic gpio_ios_2_25_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_25_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 84 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_25_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_25_sel_addressed) begin
        gpio_ios_2_25_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_25_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_25_sel),
    .out_o(gpio_ios_o[2][25])
  );

  logic [1:0] gpio_ios_0_26_sel;
  logic gpio_ios_0_26_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_26_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 88 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_26_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_26_sel_addressed) begin
        gpio_ios_0_26_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_26_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g26
    }),
    .sel_i(gpio_ios_0_26_sel),
    .out_o(gpio_ios_o[0][26])
  );

  logic [1:0] gpio_ios_1_26_sel;
  logic gpio_ios_1_26_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_26_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 88 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_26_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_26_sel_addressed) begin
        gpio_ios_1_26_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_26_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_26_sel),
    .out_o(gpio_ios_o[1][26])
  );

  logic [1:0] gpio_ios_2_26_sel;
  logic gpio_ios_2_26_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_26_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 88 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_26_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_26_sel_addressed) begin
        gpio_ios_2_26_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_26_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_26_sel),
    .out_o(gpio_ios_o[2][26])
  );

  logic [1:0] gpio_ios_0_27_sel;
  logic gpio_ios_0_27_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_27_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 88 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_27_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_27_sel_addressed) begin
        gpio_ios_0_27_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_27_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      rph_g27
    }),
    .sel_i(gpio_ios_0_27_sel),
    .out_o(gpio_ios_o[0][27])
  );

  logic [1:0] gpio_ios_1_27_sel;
  logic gpio_ios_1_27_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_27_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 92 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_27_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_27_sel_addressed) begin
        gpio_ios_1_27_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_27_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_27_sel),
    .out_o(gpio_ios_o[1][27])
  );

  logic [1:0] gpio_ios_2_27_sel;
  logic gpio_ios_2_27_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_27_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 92 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_27_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_27_sel_addressed) begin
        gpio_ios_2_27_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_27_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_27_sel),
    .out_o(gpio_ios_o[2][27])
  );

  logic [1:0] gpio_ios_0_28_sel;
  logic gpio_ios_0_28_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_28_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 92 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_28_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_28_sel_addressed) begin
        gpio_ios_0_28_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_28_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_0_28_sel),
    .out_o(gpio_ios_o[0][28])
  );

  logic [1:0] gpio_ios_1_28_sel;
  logic gpio_ios_1_28_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_28_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 92 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_28_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_28_sel_addressed) begin
        gpio_ios_1_28_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_28_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_28_sel),
    .out_o(gpio_ios_o[1][28])
  );

  logic [1:0] gpio_ios_2_28_sel;
  logic gpio_ios_2_28_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_28_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 96 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_28_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_28_sel_addressed) begin
        gpio_ios_2_28_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_28_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_28_sel),
    .out_o(gpio_ios_o[2][28])
  );

  logic [1:0] gpio_ios_0_29_sel;
  logic gpio_ios_0_29_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_29_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 96 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_29_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_29_sel_addressed) begin
        gpio_ios_0_29_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_29_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_0_29_sel),
    .out_o(gpio_ios_o[0][29])
  );

  logic [1:0] gpio_ios_1_29_sel;
  logic gpio_ios_1_29_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_29_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 96 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_29_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_29_sel_addressed) begin
        gpio_ios_1_29_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_29_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_29_sel),
    .out_o(gpio_ios_o[1][29])
  );

  logic [1:0] gpio_ios_2_29_sel;
  logic gpio_ios_2_29_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_29_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 96 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_29_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_29_sel_addressed) begin
        gpio_ios_2_29_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_29_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_29_sel),
    .out_o(gpio_ios_o[2][29])
  );

  logic [1:0] gpio_ios_0_30_sel;
  logic gpio_ios_0_30_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_30_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 100 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_30_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_30_sel_addressed) begin
        gpio_ios_0_30_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_30_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_0_30_sel),
    .out_o(gpio_ios_o[0][30])
  );

  logic [1:0] gpio_ios_1_30_sel;
  logic gpio_ios_1_30_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_30_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 100 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_30_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_30_sel_addressed) begin
        gpio_ios_1_30_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_30_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_30_sel),
    .out_o(gpio_ios_o[1][30])
  );

  logic [1:0] gpio_ios_2_30_sel;
  logic gpio_ios_2_30_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_30_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 100 &
    reg_be[2] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_30_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_30_sel_addressed) begin
        gpio_ios_2_30_sel <= reg_wdata[16+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_30_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_30_sel),
    .out_o(gpio_ios_o[2][30])
  );

  logic [1:0] gpio_ios_0_31_sel;
  logic gpio_ios_0_31_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_0_31_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 100 &
    reg_be[3] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_0_31_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_0_31_sel_addressed) begin
        gpio_ios_0_31_sel <= reg_wdata[24+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_0_31_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_0_31_sel),
    .out_o(gpio_ios_o[0][31])
  );

  logic [1:0] gpio_ios_1_31_sel;
  logic gpio_ios_1_31_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_1_31_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 104 &
    reg_be[0] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_1_31_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_1_31_sel_addressed) begin
        gpio_ios_1_31_sel <= reg_wdata[0+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_1_31_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_1_31_sel),
    .out_o(gpio_ios_o[1][31])
  );

  logic [1:0] gpio_ios_2_31_sel;
  logic gpio_ios_2_31_sel_addressed;

  // Register addresses of 0x800 to 0xfff are block IO selectors, which are packed with 4 per 32-bit word.
  assign gpio_ios_2_31_sel_addressed =
    reg_addr[RegAddrWidth-1] == 1'b1 &
    reg_addr[RegAddrWidth-2:0] == 104 &
    reg_be[1] == 1'b1;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // Select second input by default so that pins are connected to the first block that is specified in the configuration.
      gpio_ios_2_31_sel <= 2'b10;
    end else begin
      if (reg_we & gpio_ios_2_31_sel_addressed) begin
        gpio_ios_2_31_sel <= reg_wdata[8+:2];
      end
    end
  end

  prim_onehot_mux #(
    .Width(1),
    .Inputs(2)
  ) gpio_ios_2_31_mux (
    .clk_i,
    .rst_ni,
    .in_i({
      1'b0,
      1'b0
    }),
    .sel_i(gpio_ios_2_31_sel),
    .out_o(gpio_ios_o[2][31])
  );

  // Combining inputs for combinable inouts
  assign i2c_scl_o[0] =
    (scl0_sel == 2 ? scl0 : 1) &
    (rph_g1_sel == 2 ? rph_g1 : 1);
  assign i2c_scl_o[1] =
    (scl1_sel == 2 ? scl1 : 1) &
    (rph_g3_scl_sel == 2 ? rph_g3_scl : 1) &
    (mb6_sel == 2 ? mb6 : 1);
  assign i2c_sda_o[0] =
    (sda0_sel == 2 ? sda0 : 1) &
    (rph_g0_sel == 2 ? rph_g0 : 1);
  assign i2c_sda_o[1] =
    (sda1_sel == 2 ? sda1 : 1) &
    (rph_g2_sda_sel == 2 ? rph_g2_sda : 1) &
    (mb5_sel == 2 ? mb5 : 1);
endmodule
