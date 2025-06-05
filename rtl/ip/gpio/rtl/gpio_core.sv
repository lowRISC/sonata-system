// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module gpio_core #(
  parameter int unsigned GpiWidth  = 8,
  parameter int unsigned GpoWidth  = 16,
  parameter int unsigned AddrWidth = 32,
  parameter int unsigned DataWidth = 32,
  parameter int unsigned RegAddr   = 6,
  parameter int unsigned DbncWidth = 10
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic                 device_req_i,
  input  logic [AddrWidth-1:0] device_addr_i,
  input  logic                 device_we_i,
  input  logic [3:0]           device_be_i,
  input  logic [DataWidth-1:0] device_wdata_i,
  output logic                 device_rvalid_o,
  output logic [DataWidth-1:0] device_rdata_o,

  input  logic [GpiWidth-1:0] gp_i,
  output logic [GpoWidth-1:0] gp_o,
  output logic [GpoWidth-1:0] gp_o_en,

  output logic                pcint_o
);

  localparam int unsigned GPIO_OUT_REG      = 32'h00;
  localparam int unsigned GPIO_IN_REG       = 32'h04;
  localparam int unsigned GPIO_IN_DBNC_REG  = 32'h08;
  localparam int unsigned GPIO_OUT_EN_REG   = 32'h0C;
  localparam int unsigned GPIO_CTRL_REG     = 32'h10;
  localparam int unsigned GPIO_STATUS_REG   = 32'h14;
  localparam int unsigned GPIO_PIN_MASK_REG = 32'h18;

  logic [RegAddr-1:0] reg_addr;

  logic [GpiWidth-1:0] gp_i_sync;
  logic [GpiWidth-1:0] gp_i_dbnc;
  logic [GpoWidth-1:0] gp_o_d;
  logic [GpoWidth-1:0] gp_o_en_d;

  logic gp_o_sel;
  logic gp_o_en_sel;
  logic ctrl_sel;
  logic status_sel;
  logic pcint_mask_sel;

  logic [RegAddr-3:0] rd_reg_idx;

  logic [DbncWidth-1:0] dbnc_cnt;
  logic dbnc_step;

  logic [GpiWidth-1:0] pcint_mask, pcint_mask_d;

  logic pcint_triggered;
  logic pcint_status, pcint_status_d;
  logic pcint_enable, pcint_enable_d;
  logic [1:0] pcint_mode, pcint_mode_d;
  logic pcint_i_sel, pcint_i_sel_d;

  // Shared counter to generate the step-pulses for input debouncers.
  // Prefer to use an extra flop rather than have to AND all the bits
  // together as the wider Sonata system is very logic-heavy.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dbnc_cnt <= '0;
    end else begin
      dbnc_cnt <= dbnc_cnt[DbncWidth-1] ? '0 : dbnc_cnt + 1;
    end
  end

  assign dbnc_step = dbnc_cnt[DbncWidth-1];

  // Instantiate an input synchroniser wide enough for all GP inputs.
  prim_flop_2sync #(
    .Width(GpiWidth),
    .ResetValue('0)
  ) u_sync (
    .clk_i,
    .rst_ni,
    .d_i(gp_i),
    .q_o(gp_i_sync)
  );

  // Instantiate step-based debouncers for all GP inputs.
  for (genvar i = 0; i < GpiWidth; i++) begin
    debounce_step dbnc (
      .clk_i,
      .rst_ni,
      .step_i(dbnc_step),
      .btn_i(gp_i_sync[i]),
      .btn_o(gp_i_dbnc[i])
    );
  end

  // Instantiated Pin-Change Interrupt (PCINT) unit wide enough for all inputs.
  pcint #(
    .Width(GpiWidth)
  ) u_pcint (
    .clk_i,
    .rst_ni,
    .mode_i(pcint_mode),
    .i_sel_i(pcint_i_sel),
    .pin_mask_i(pcint_mask),
    .sync_i(gp_i_sync),
    .dbnc_i(gp_i_dbnc),
    .pcint_o(pcint_triggered)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      gp_o              <= '0;
      gp_o_en           <= '0;
      device_rvalid_o   <= '0;
      pcint_mask        <= '0;
      pcint_enable      <= '0;
      pcint_mode        <= '0;
      pcint_i_sel       <= '0;
      pcint_status      <= '0;
      rd_reg_idx        <= '0;
    end else begin
      gp_o         <= (gp_o_sel && device_we_i)       ? gp_o_d         : gp_o;
      gp_o_en      <= (gp_o_en_sel && device_we_i)    ? gp_o_en_d      : gp_o_en;
      pcint_mask   <= (pcint_mask_sel && device_we_i) ? pcint_mask_d   : pcint_mask;
      pcint_enable <= (ctrl_sel && device_we_i)       ? pcint_enable_d : pcint_enable;
      pcint_mode   <= (ctrl_sel && device_we_i)       ? pcint_mode_d   : pcint_mode;
      pcint_i_sel  <= (ctrl_sel && device_we_i)       ? pcint_i_sel_d  : pcint_i_sel;

      // pcint_status is set by hardware and only cleared when a '1' is written
      if (pcint_triggered) begin
        pcint_status <= 1'b1;
      end else if (status_sel && device_we_i && pcint_status_d) begin
        pcint_status <= 1'b0;
      end else begin
        pcint_status <= pcint_status;
      end

      device_rvalid_o <= device_req_i && !device_we_i;
      rd_reg_idx <= reg_addr[RegAddr-1:2];
    end
  end

  logic [3:0] unused_o_device_be;
  logic [3:0] unused_i_device_be;

  // Assign gp_o_d regarding to device_be_i and GpoWidth.
  for (genvar i_byte = 0; i_byte < 4; ++i_byte) begin : g_gp_o_d;
    if (i_byte * 8 < GpoWidth) begin : gen_gp_o_d_inner
      localparam int gpo_byte_end = (i_byte + 1) * 8 <= GpoWidth ? (i_byte + 1) * 8 : GpoWidth;
      assign gp_o_d[gpo_byte_end - 1 : i_byte * 8] =
        device_be_i[i_byte] ? device_wdata_i[gpo_byte_end - 1 : i_byte * 8] :
                              gp_o[gpo_byte_end - 1 : i_byte * 8];
      assign gp_o_en_d[gpo_byte_end - 1 : i_byte * 8] =
        device_be_i[i_byte] ? device_wdata_i[gpo_byte_end - 1 : i_byte * 8] :
                              gp_o_en[gpo_byte_end - 1 : i_byte * 8];
      assign unused_o_device_be[i_byte] = 0;
    end else begin : gen_unused_device_be
      assign unused_o_device_be[i_byte] = device_be_i[i_byte];
    end
  end

  // Assign pcint_mask_d regarding to device_be_i and GpiWidth.
  for (genvar i_byte = 0; i_byte < 4; ++i_byte) begin : g_pcint_mask_d;
    if (i_byte * 8 < GpiWidth) begin : gen_pcint_mask_d_inner
      localparam int gpi_byte_end = (i_byte + 1) * 8 <= GpiWidth ? (i_byte + 1) * 8 : GpiWidth;
      assign pcint_mask_d[gpi_byte_end - 1 : i_byte * 8] =
        device_be_i[i_byte] ? device_wdata_i[gpi_byte_end - 1 : i_byte * 8] :
                              pcint_mask[gpi_byte_end - 1 : i_byte * 8];
      assign unused_i_device_be[i_byte] = 0;
    end else begin : gen_unused_device_be
      assign unused_i_device_be[i_byte] = device_be_i[i_byte];
    end
  end

  assign pcint_status_d = device_be_i[3] ? device_wdata_i[31]  : 1'b0; // special case
  assign pcint_enable_d = device_be_i[3] ? device_wdata_i[31]  : pcint_enable;
  assign pcint_i_sel_d  = device_be_i[0] ? device_wdata_i[3]   : pcint_i_sel;
  assign pcint_mode_d   = device_be_i[0] ? device_wdata_i[1:0] : pcint_mode;

  // Decode write requests.
  assign reg_addr       = device_addr_i[RegAddr-1:0];
  assign gp_o_sel       = device_req_i & (reg_addr == GPIO_OUT_REG[RegAddr-1:0]);
  assign gp_o_en_sel    = device_req_i & (reg_addr == GPIO_OUT_EN_REG[RegAddr-1:0]);
  assign ctrl_sel       = device_req_i & (reg_addr == GPIO_CTRL_REG[RegAddr-1:0]);
  assign status_sel     = device_req_i & (reg_addr == GPIO_STATUS_REG[RegAddr-1:0]);
  assign pcint_mask_sel = device_req_i & (reg_addr == GPIO_PIN_MASK_REG[RegAddr-1:0]);

  // Assign device_rdata_o according to request type.
  always_comb begin
    if (!device_rvalid_o)
      device_rdata_o = {(DataWidth){1'b0}};
    else if (rd_reg_idx == GPIO_OUT_REG[RegAddr-1:2])
      device_rdata_o = {{(DataWidth - GpoWidth){1'b0}}, gp_o};
    else if (rd_reg_idx == GPIO_IN_REG[RegAddr-1:2])
      device_rdata_o = {{(DataWidth - GpiWidth){1'b0}}, gp_i_sync};
    else if (rd_reg_idx == GPIO_IN_DBNC_REG[RegAddr-1:2])
      device_rdata_o = {{(DataWidth - GpiWidth){1'b0}}, gp_i_dbnc};
    else if (rd_reg_idx == GPIO_OUT_EN_REG[RegAddr-1:2])
      device_rdata_o = {{(DataWidth - GpoWidth){1'b0}}, gp_o_en};
    else if (rd_reg_idx == GPIO_CTRL_REG[RegAddr-1:2])
      device_rdata_o = {pcint_enable, 27'b0, pcint_i_sel, 1'b0, pcint_mode};
    else if (rd_reg_idx == GPIO_STATUS_REG[RegAddr-1:2])
      device_rdata_o = {pcint_status, 31'b0};
    else if (rd_reg_idx == GPIO_PIN_MASK_REG[RegAddr-1:2])
      device_rdata_o = {{(DataWidth - GpiWidth){1'b0}}, pcint_mask};
    else
      device_rdata_o = {(DataWidth){1'b0}};
  end

  assign pcint_o = pcint_enable & pcint_status;

  // Unused signals.
  if (AddrWidth > RegAddr) begin : g_unused_addr_bits
    logic [AddrWidth-1-RegAddr:0]  unused_device_addr;
    assign unused_device_addr  = device_addr_i[AddrWidth-1:RegAddr];
  end

  if (DataWidth > GpoWidth) begin : g_unused_gpo_bits
    logic [DataWidth-1-GpoWidth:0] unused_device_wdata;
    assign unused_device_wdata = device_wdata_i[DataWidth-1:GpoWidth];
  end
endmodule
