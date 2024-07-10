// Copyright (C) Microsoft Corporation. All rights reserved.

module msftDvIp_mmreg (
  input  logic         clk_i,
  input  logic         rstn_i,

  input  logic         reg_en_i,
  input  logic [31:0]  reg_addr_i,
  input  logic [31:0]  reg_wdata_i,
  input  logic         reg_we_i,
  output logic [31:0]  reg_rdata_o,
  output logic         reg_ready_o,
                    
  input  logic [63:0]  mmreg_coreout_i,
  output logic [127:0] mmreg_corein_o,

  output logic         tbre_intr_o
);


  //=================================================
  // Registers/Wires
  //=================================================
  logic [31:0]      tbre_start_addr;
  logic [31:0]      tbre_end_addr;
  logic             tbre_go;
  logic [30:0]      tbre_epoch;
  logic             tbre_done, tbre_stat_q, tbre_stat;

  logic             tbre_intr_stat, tbre_intr_en;

  logic [4:0]       dbg_fifo_depth;
  logic [4:0]       dbg_fifo_ext_wr_ptr, dbg_fifo_ext_rd_ptr;
  logic [3:0]       dbg_fifo_wr_ptr, dbg_fifo_rd_ptr;
  logic             dbg_fifo_empty, dbg_fifo_full;
  logic [15:0][7:0] dbg_fifo_mem;
  logic [7:0]       dbg_fifo_wr_data, dbg_fifo_rd_data;
  logic             dbg_fifo_wr_en, dbg_fifo_rd_en;

  logic             wr_op, rd_op;


  //=================================================
  // Assignements
  //=================================================
  assign mmreg_corein_o = {63'h0, tbre_go, tbre_end_addr, tbre_start_addr};
  assign tbre_stat      = mmreg_coreout_i[0];
  assign tbre_done      = tbre_stat_q & ~tbre_stat;

  assign tbre_intr_o    = tbre_intr_stat & tbre_intr_en;

  assign wr_op = reg_en_i & reg_we_i;
  assign rd_op = reg_en_i & ~reg_we_i;

  assign reg_ready_o = 1'b1;

  //=================================================
  // Write registers
  //=================================================
  always @(posedge clk_i or negedge rstn_i)
  begin
    if(~rstn_i) begin
      tbre_start_addr <= 32'h0;
      tbre_end_addr   <= 32'h0;
      tbre_go         <= 1'b0;
      tbre_epoch      <= 31'h0;
      tbre_stat_q     <= 1'b0;
      tbre_intr_en    <= 1'b0;
      tbre_intr_stat  <= 1'b0;

    end else begin
      if(wr_op) begin
        case(reg_addr_i[7:2])
          6'h0: tbre_start_addr <= reg_wdata_i;
          6'h1: tbre_end_addr   <= reg_wdata_i;
          6'h5: tbre_intr_en    <= reg_wdata_i[0];
          default:;
        endcase
      end

      if (wr_op && (reg_addr_i[7:2] == 6'h2))
        tbre_go <= 1'b1;
      else
        tbre_go <= 1'b0;

      tbre_stat_q <= tbre_stat;
      
      if (tbre_done)
        tbre_epoch <= tbre_epoch + 1;

      if (tbre_done) tbre_intr_stat <= 1'b1;
      else if (tbre_intr_en) tbre_intr_stat <= 1'b0;   // auto clear

    end
  end

  //=================================================
  // Read registers
  //=================================================
  always @(posedge clk_i or negedge rstn_i)
  begin
    if(~rstn_i) begin
      reg_rdata_o <= 32'h0000_0000;
    end else begin
      if (rd_op) begin
        casez(reg_addr_i[7:2]) 
          6'h0:    reg_rdata_o <= tbre_start_addr;
          6'h1:    reg_rdata_o <= tbre_end_addr;
          6'h2:    reg_rdata_o <= {16'h5500, 15'h0, tbre_go};
          6'h3:    reg_rdata_o <= {tbre_epoch, tbre_stat};
          6'h4:    reg_rdata_o <= {31'h0, tbre_intr_stat};
          6'h5:    reg_rdata_o <= {31'h0, tbre_intr_en};
          6'h10:   reg_rdata_o <= {23'h0, dbg_fifo_empty, dbg_fifo_rd_data};
          6'h11:   reg_rdata_o <= {22'h0, dbg_fifo_full, dbg_fifo_empty, 3'h0, dbg_fifo_depth};
          default: reg_rdata_o <= 32'h0000_0000;
        endcase
      end
    end
  end

  //=================================================
  // Debug (fast printf) FIFO
  //=================================================
  assign dbg_fifo_wr_ptr = dbg_fifo_ext_wr_ptr[3:0];
  assign dbg_fifo_rd_ptr = dbg_fifo_ext_rd_ptr[3:0];

  assign dbg_fifo_rd_data = dbg_fifo_mem[dbg_fifo_rd_ptr];
  assign dbg_fifo_wr_data = reg_wdata_i[7:0];

  assign dbg_fifo_depth = dbg_fifo_ext_wr_ptr - dbg_fifo_ext_rd_ptr;
  assign dbg_fifo_empty = (dbg_fifo_depth == 0);
  assign dbg_fifo_full  = (dbg_fifo_depth >= 16);

  assign dbg_fifo_wr_en = wr_op & (reg_addr_i[7:2] == 6'h10);
  assign dbg_fifo_rd_en = rd_op & (reg_addr_i[7:2] == 6'h10);

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      dbg_fifo_ext_rd_ptr <= 'h0;
      dbg_fifo_ext_wr_ptr <= 'h0;
    end else begin
      // FIFO size is power-of-2
      if (dbg_fifo_rd_en & ~dbg_fifo_empty) dbg_fifo_ext_rd_ptr <= dbg_fifo_ext_rd_ptr + 1;
      if (dbg_fifo_wr_en & ~dbg_fifo_full)  dbg_fifo_ext_wr_ptr <= dbg_fifo_ext_wr_ptr + 1;
    end
  end

  for (genvar i= 0; i < 16; i++) begin : gen_fifo_mem
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        dbg_fifo_mem[i]  <= 0;
      end else begin
        if (dbg_fifo_wr_en & ~dbg_fifo_full && (i == dbg_fifo_wr_ptr))
          dbg_fifo_mem[i] <= dbg_fifo_wr_data;
      end
    end
  end // generate

  logic _unused_signals;
  assign _unused_signals = |reg_addr_i[31:8] | |reg_addr_i[1:0] | |mmreg_coreout_i[63:1];

endmodule
