// Copyright Microsoft Corporation
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0


/**
 * Load Store Unit
 *
 * Load Store Unit, used to eliminate multiple access during processor stalls,
 * and to align bytes and halfwords.
 */

`include "prim_assert.sv"
`include "dv_fcov_macros.svh"

module ibex_load_store_unit import ibex_pkg::*; import cheri_pkg::*; #(
  parameter bit CHERIoTEn = 1'b1,
  parameter bit MemCapFmt = 1'b0,
  parameter bit CheriTBRE = 1'b0
)(
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         cheri_pmode_i,

  // data interface
  output logic         data_req_o,
  output logic         data_is_cap_o,
  input  logic         data_gnt_i,
  input  logic         data_rvalid_i,
  input  logic         data_err_i,
  input  logic         data_pmp_err_i,

  output logic [31:0]  data_addr_o,
  output logic         data_we_o,
  output logic [3:0]   data_be_o,
  output logic [32:0]  data_wdata_o,         // kliu
  input  logic [32:0]  data_rdata_i,         // kliu

  // signals to/from ID/EX stage
  input  logic         lsu_we_i,             // write enable                     -> from ID/EX
  input  logic         lsu_is_cap_i,         // kliu
  input  logic         lsu_is_intl_i,        // kliu: "internal" requests from cheri_ex
  input  logic         lsu_cheri_err_i,      // kliu
  input  logic [1:0]   lsu_type_i,           // data type: word, half word, byte -> from ID/EX
  input  logic [32:0]  lsu_wdata_i,          // data to write to memory          -> from ID/EX
  input  reg_cap_t     lsu_wcap_i,           // kliu
  input  logic [3:0]   lsu_lc_clrperm_i,
  input  logic         lsu_sign_ext_i,       // sign extension                   -> from ID/EX

  output reg_cap_t     lsu_rcap_o,           // kliu
  output logic [32:0]  lsu_rdata_o,          // requested data                   -> to ID/EX
  output logic         lsu_rdata_valid_o,
  input  logic         lsu_req_i,            // data request                     -> from ID/EX

  input  logic [31:0]  lsu_addr_i,           // address computed in ALU          -> from ID/EX

  output logic         lsu_addr_incr_req_o,  // request address increment for
                                              // misaligned accesses              -> to ID/EX
  output logic [31:0]  addr_last_o,          // address of last transaction      -> to controller
                                              // -> mtval
                                              // -> AGU for misaligned accesses

  output logic         lsu_req_done_o,       // Signals that data request is complete
                                              // (only need to await final data
                                              // response)                        -> to ID/EX
  output logic         lsu_req_done_intl_o,
  output logic         lsu_resp_valid_o,     // LSU has response from transaction -> to ID/EX & WB
  output logic         lsu_resp_valid_intl_o, // LSU response is for a cheri cap load/store -> to cheri_ex
  output logic         lsu_resp_is_wr_o,

  // TBRE related signals
  input  logic         tbre_lsu_req_i,
  input  logic         cpu_lsu_dec_i,
  output logic         lsu_tbre_sel_o,        // request-side selection signal
  output logic         lsu_tbre_addr_incr_req_o,  // request address increment for
  output logic [32:0]  lsu_tbre_raw_lsw_o,
  output logic         lsu_tbre_req_done_o,
  output logic         lsu_tbre_resp_valid_o, // response from transaction -> to TBRE 
  output logic         lsu_tbre_resp_err_o,

  // exception signals
  output logic         load_err_o,
  output logic         store_err_o,
  output logic         lsu_err_is_cheri_o,
  output logic         lsu_resp_err_intl_o,

  output logic         busy_o,
  output logic         busy_tbre_o,

  output logic         perf_load_o,
  output logic         perf_store_o
);

  logic [31:0]  data_addr;
  logic [31:0]  data_addr_w_aligned;
  logic [31:0]  addr_last_q, addr_last_d;

  logic         addr_update;
  logic         ctrl_update;
  logic         rdata_update;
  logic [31:8]  rdata_q;
  logic [1:0]   rdata_offset_q;
  logic [1:0]   data_type_q;
  logic         data_sign_ext_q;
  logic         data_we_q;

  logic [1:0]   data_offset;   // mux control for data to be written to memory

  logic [3:0]   data_be;
  logic [32:0]  data_wdata;

  logic [32:0]  data_rdata_ext;

  logic [32:0]  rdata_w_ext; // word realignment for misaligned loads
  logic [31:0]  rdata_h_ext; // sign extension for half words
  logic [31:0]  rdata_b_ext; // sign extension for bytes

  logic         split_misaligned_access;
  logic         handle_misaligned_q, handle_misaligned_d; // high after receiving grant for first
                                                          // part of a misaligned access
  logic         pmp_err_q, pmp_err_d;
  logic         lsu_err_q, lsu_err_d;
  logic         data_or_pmp_err;

  logic         resp_is_cap_q, resp_is_intl_q;
  logic         cheri_err_d, cheri_err_q;
  logic [3:0]   resp_lc_clrperm_q;
  logic         cur_req_is_tbre, req_is_tbre_q;
  logic         resp_is_tbre_q;
  logic         tbre_req_good;
  logic         outstanding_resp_q, resp_wait;
  logic         lsu_resp_valid;
  logic         lsu_go;
  logic         addr_incr_req;

  ls_fsm_e ls_fsm_cs, ls_fsm_ns;

  cap_rx_fsm_t cap_rx_fsm_q, cap_rx_fsm_d;

  logic         cap_lsw_err_q;
  logic [32:0]  cap_lsw_q;

  assign data_addr   = lsu_addr_i;
  assign data_offset = (cheri_pmode_i & lsu_is_cap_i) ? 2'b00 : data_addr[1:0];

  ///////////////////
  // BE generation //
  ///////////////////

  always_comb begin
    if (CHERIoTEn & cheri_pmode_i & lsu_is_cap_i)
      data_be = 4'b1111;        // caps are always word aligned
    else begin
      unique case (lsu_type_i) // Data type 00 Word, 01 Half word, 11,10 byte
        2'b00: begin // Writing a word
          if (!handle_misaligned_q) begin // first part of potentially misaligned transaction
            unique case (data_offset)
              2'b00:   data_be = 4'b1111;
              2'b01:   data_be = 4'b1110;
              2'b10:   data_be = 4'b1100;
              2'b11:   data_be = 4'b1000;
              default: data_be = 4'b1111;
            endcase // case (data_offset)
          end else begin // second part of misaligned transaction
            unique case (data_offset)
              2'b00:   data_be = 4'b0000; // this is not used, but included for completeness
              2'b01:   data_be = 4'b0001;
              2'b10:   data_be = 4'b0011;
              2'b11:   data_be = 4'b0111;
              default: data_be = 4'b1111;
            endcase // case (data_offset)
          end
        end

        2'b01: begin // Writing a half word
          if (!handle_misaligned_q) begin // first part of potentially misaligned transaction
            unique case (data_offset)
              2'b00:   data_be = 4'b0011;
              2'b01:   data_be = 4'b0110;
              2'b10:   data_be = 4'b1100;
              2'b11:   data_be = 4'b1000;
              default: data_be = 4'b1111;
            endcase // case (data_offset)
          end else begin // second part of misaligned transaction
            data_be = 4'b0001;
          end
        end

        2'b10,
        2'b11: begin // Writing a byte
          unique case (data_offset)
            2'b00:   data_be = 4'b0001;
            2'b01:   data_be = 4'b0010;
            2'b10:   data_be = 4'b0100;
            2'b11:   data_be = 4'b1000;
            default: data_be = 4'b1111;
          endcase // case (data_offset)
        end

        default:     data_be = 4'b1111;
      endcase // case (lsu_type_i)
    end  // if lsu_cap_i
  end

  /////////////////////
  // WData alignment //
  /////////////////////

  // prepare data to be written to the memory
  // we handle misaligned accesses, half word and byte accesses here
  if (~MemCapFmt) begin : gen_memcap_wr_fmt0
    always_comb begin
      if (CHERIoTEn & cheri_pmode_i & lsu_is_cap_i && (ls_fsm_cs == CTX_WAIT_GNT2))
        data_wdata = reg2memcap_fmt0(lsu_wcap_i);
      else if (CHERIoTEn & cheri_pmode_i & lsu_is_cap_i)
        data_wdata = lsu_wdata_i;
      else begin
        unique case (data_offset)
          2'b00:   data_wdata =  lsu_wdata_i[32:0];
          2'b01:   data_wdata = {1'b0, lsu_wdata_i[23:0], lsu_wdata_i[31:24]};
          2'b10:   data_wdata = {1'b0, lsu_wdata_i[15:0], lsu_wdata_i[31:16]};
          2'b11:   data_wdata = {1'b0, lsu_wdata_i[ 7:0], lsu_wdata_i[31: 8]};
          default: data_wdata =  lsu_wdata_i[32:0];
        endcase // case (data_offset)
      end
    end
  end else begin : gen_memcap_wr_fmt1
    logic [65:0] mem_capaddr;
    assign mem_capaddr = reg2mem_fmt1(lsu_wcap_i, lsu_wdata_i);

    always_comb begin
      if (CHERIoTEn & lsu_is_cap_i && (ls_fsm_cs == CTX_WAIT_GNT2))
        data_wdata = mem_capaddr[65:33];
      else if (CHERIoTEn & lsu_is_cap_i)
        data_wdata = mem_capaddr[32:0];
      else begin
        unique case (data_offset)
          2'b00:   data_wdata =  lsu_wdata_i[32:0];
          2'b01:   data_wdata = {1'b0, lsu_wdata_i[23:0], lsu_wdata_i[31:24]};
          2'b10:   data_wdata = {1'b0, lsu_wdata_i[15:0], lsu_wdata_i[31:16]};
          2'b11:   data_wdata = {1'b0, lsu_wdata_i[ 7:0], lsu_wdata_i[31: 8]};
          default: data_wdata =  lsu_wdata_i[32:0];
        endcase // case (data_offset)
      end
    end
  end
  /////////////////////
  // RData alignment //
  /////////////////////

  // register for unaligned rdata
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rdata_q <= '0;
    end else if (rdata_update) begin
      rdata_q <= data_rdata_i[31:8];
    end
  end

  // registers for transaction control
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rdata_offset_q  <= 2'h0;
      data_type_q     <= 2'h0;
      data_sign_ext_q <= 1'b0;
      data_we_q       <= 1'b0;
    end else if (ctrl_update) begin
      rdata_offset_q  <= data_offset;
      data_type_q     <= lsu_type_i;
      data_sign_ext_q <= lsu_sign_ext_i;
      data_we_q       <= lsu_we_i;
    end
  end

  // Store last address for mtval + AGU for misaligned transactions.  Do not update in case of
  // errors, mtval needs the (first) failing address.  Where an aligned access or the first half of
  // a misaligned access sees an error provide the calculated access address. For the second half of
  // a misaligned access provide the word aligned address of the second half.
  assign addr_last_d = addr_incr_req ? data_addr_w_aligned : data_addr;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      addr_last_q <= '0;
    end else if (addr_update & ~cur_req_is_tbre) begin
      addr_last_q <= addr_last_d;
    end
  end

  // take care of misaligned words
  always_comb begin
    unique case (rdata_offset_q)
      2'b00:   rdata_w_ext =  data_rdata_i[32:0];
      2'b01:   rdata_w_ext = {1'b0, data_rdata_i[ 7:0], rdata_q[31:8]};
      2'b10:   rdata_w_ext = {1'b0, data_rdata_i[15:0], rdata_q[31:16]};
      2'b11:   rdata_w_ext = {1'b0, data_rdata_i[23:0], rdata_q[31:24]};
      default: rdata_w_ext =  data_rdata_i[32:0];
    endcase
  end

  ////////////////////
  // Sign extension //
  ////////////////////

  // sign extension for half words
  always_comb begin
    unique case (rdata_offset_q)
      2'b00: begin
        if (!data_sign_ext_q) begin
          rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
        end else begin
          rdata_h_ext = {{16{data_rdata_i[15]}}, data_rdata_i[15:0]};
        end
      end

      2'b01: begin
        if (!data_sign_ext_q) begin
          rdata_h_ext = {16'h0000, data_rdata_i[23:8]};
        end else begin
          rdata_h_ext = {{16{data_rdata_i[23]}}, data_rdata_i[23:8]};
        end
      end

      2'b10: begin
        if (!data_sign_ext_q) begin
          rdata_h_ext = {16'h0000, data_rdata_i[31:16]};
        end else begin
          rdata_h_ext = {{16{data_rdata_i[31]}}, data_rdata_i[31:16]};
        end
      end

      2'b11: begin
        if (!data_sign_ext_q) begin
          rdata_h_ext = {16'h0000, data_rdata_i[7:0], rdata_q[31:24]};
        end else begin
          rdata_h_ext = {{16{data_rdata_i[7]}}, data_rdata_i[7:0], rdata_q[31:24]};
        end
      end

      default: rdata_h_ext = {16'h0000, data_rdata_i[15:0]};
    endcase // case (rdata_offset_q)
  end

  // sign extension for bytes
  always_comb begin
    unique case (rdata_offset_q)
      2'b00: begin
        if (!data_sign_ext_q) begin
          rdata_b_ext = {24'h00_0000, data_rdata_i[7:0]};
        end else begin
          rdata_b_ext = {{24{data_rdata_i[7]}}, data_rdata_i[7:0]};
        end
      end

      2'b01: begin
        if (!data_sign_ext_q) begin
          rdata_b_ext = {24'h00_0000, data_rdata_i[15:8]};
        end else begin
          rdata_b_ext = {{24{data_rdata_i[15]}}, data_rdata_i[15:8]};
        end
      end

      2'b10: begin
        if (!data_sign_ext_q) begin
          rdata_b_ext = {24'h00_0000, data_rdata_i[23:16]};
        end else begin
          rdata_b_ext = {{24{data_rdata_i[23]}}, data_rdata_i[23:16]};
        end
      end

      2'b11: begin
        if (!data_sign_ext_q) begin
          rdata_b_ext = {24'h00_0000, data_rdata_i[31:24]};
        end else begin
          rdata_b_ext = {{24{data_rdata_i[31]}}, data_rdata_i[31:24]};
        end
      end

      default: rdata_b_ext = {24'h00_0000, data_rdata_i[7:0]};
    endcase // case (rdata_offset_q)
  end

  // select word, half word or byte sign extended version
  always_comb begin
    unique case (data_type_q)
      2'b00:       data_rdata_ext = rdata_w_ext;
      2'b01:       data_rdata_ext = {1'b0, rdata_h_ext};
      2'b10,2'b11: data_rdata_ext = {1'b0, rdata_b_ext};
      default:     data_rdata_ext = rdata_w_ext;
    endcase // case (data_type_q)
  end

  /////////////
  // LSU FSM //
  /////////////

  // check for misaligned accesses that need to be split into two word-aligned accesses
  assign split_misaligned_access =
      ((lsu_type_i == 2'b00) && (data_offset != 2'b00)) || // misaligned word access
      ((lsu_type_i == 2'b01) && (data_offset == 2'b11));   // misaligned half-word access


  // FSM
  always_comb begin
    ls_fsm_ns       = ls_fsm_cs;

    data_req_o          = 1'b0;
    addr_incr_req     = 1'b0;
    handle_misaligned_d = handle_misaligned_q;
    pmp_err_d           = pmp_err_q;
    lsu_err_d           = lsu_err_q;
    cheri_err_d         = cheri_err_q & cheri_pmode_i;

    addr_update         = 1'b0;
    ctrl_update         = 1'b0;
    rdata_update        = 1'b0;

    perf_load_o         = 1'b0;
    perf_store_o        = 1'b0;

    lsu_go              = 1'b0;

    unique case (ls_fsm_cs)

      IDLE: begin
        pmp_err_d   = 1'b0;
        cheri_err_d = 1'b0;

        if (CHERIoTEn & cheri_pmode_i & lsu_req_i & lsu_cheri_err_i & ~resp_wait) begin
          // cheri access error case, don't issue data_req but send error response back to WB stage
          data_req_o   = 1'b0;          
          cheri_err_d  = 1'b1;
          ctrl_update  = 1'b1;         // update ctrl/address so we can report error correctly
          addr_update  = 1'b1;
          pmp_err_d    = 1'b0;
          lsu_err_d    = 1'b0;
          perf_load_o  = 1'b0;
          lsu_go       = 1'b1;         // decision to move forward with a request
          ls_fsm_ns    = IDLE;
        end else if (CHERIoTEn & cheri_pmode_i & (lsu_req_i | tbre_req_good) & lsu_is_cap_i & ~resp_wait) begin
          // normal cap access case
          data_req_o   = 1'b1;
          cheri_err_d  = 1'b0;
          pmp_err_d    = data_pmp_err_i;
          lsu_err_d    = 1'b0;
          perf_load_o  = ~lsu_we_i;
          perf_store_o = lsu_we_i;
          lsu_go       = 1'b1;         // decision to move forward with a request

          if (data_gnt_i) begin
            ctrl_update         = 1'b1;
            addr_update         = 1'b1;
            ls_fsm_ns           = CTX_WAIT_GNT2;
          end else begin
            ls_fsm_ns           = CTX_WAIT_GNT1;
          end
        end else if ((lsu_req_i | tbre_req_good) & ~resp_wait) begin   
          // normal data access case
          data_req_o   = 1'b1;
          cheri_err_d  = 1'b0;
          pmp_err_d    = data_pmp_err_i;
          lsu_err_d    = 1'b0;
          perf_load_o  = ~lsu_we_i;
          perf_store_o = lsu_we_i;
          lsu_go       = 1'b1;         // decision to move forward with a request

          if (data_gnt_i) begin
            ctrl_update         = 1'b1;
            addr_update         = 1'b1;
            handle_misaligned_d = split_misaligned_access;
            ls_fsm_ns           = split_misaligned_access ? WAIT_RVALID_MIS : IDLE;
          end else begin
            ls_fsm_ns           = split_misaligned_access ? WAIT_GNT_MIS    : WAIT_GNT;
          end
        end

      end

      WAIT_GNT_MIS: begin
        data_req_o = 1'b1;
        // data_pmp_err_i is valid during the address phase of a request. An error will block the
        // external request and so a data_gnt_i might never be signalled. The registered version
        // pmp_err_q is only updated for new address phases and so can be used in WAIT_GNT* and
        // WAIT_RVALID* states
        if (data_gnt_i || pmp_err_q ) begin
          addr_update         = 1'b1;
          ctrl_update         = 1'b1;
          handle_misaligned_d = 1'b1;
          ls_fsm_ns           = WAIT_RVALID_MIS;
        end
      end

      WAIT_RVALID_MIS: begin
        // push out second request
        data_req_o = 1'b1;
        // tell ID/EX stage to update the address
        addr_incr_req = 1'b1;

        // first part rvalid is received, or gets a PMP error
        if (data_rvalid_i || pmp_err_q) begin
          // Update the PMP error for the second part
          pmp_err_d = data_pmp_err_i;
          // Record the error status of the first part
          lsu_err_d = data_err_i | pmp_err_q;
          // Capture the first rdata for loads
          rdata_update = ~data_we_q;
          // If already granted, wait for second rvalid
          ls_fsm_ns = data_gnt_i ? IDLE : WAIT_GNT;
          // Update the address for the second part, if no error
          addr_update = data_gnt_i & ~(data_err_i | pmp_err_q);
          // clear handle_misaligned if second request is granted
          handle_misaligned_d = ~data_gnt_i;
        end else begin
          // first part rvalid is NOT received
          if (data_gnt_i) begin
            // second grant is received
            ls_fsm_ns = WAIT_RVALID_MIS_GNTS_DONE;
            handle_misaligned_d = 1'b0;
          end
        end
      end

      WAIT_GNT: begin
        // tell ID/EX stage to update the address
        addr_incr_req = handle_misaligned_q;
        data_req_o      = 1'b1;
        if (data_gnt_i || pmp_err_q) begin
          ctrl_update         = 1'b1;
          // Update the address, unless there was an error
          addr_update         = ~lsu_err_q;
          ls_fsm_ns           = IDLE;
          handle_misaligned_d = 1'b0;
        end
      end

      WAIT_RVALID_MIS_GNTS_DONE: begin
        // tell ID/EX stage to update the address (to make sure the
        // second address can be captured correctly for mtval and PMP checking)
        addr_incr_req = 1'b1;
        // Wait for the first rvalid, second request is already granted
        if (data_rvalid_i) begin
          // Update the pmp error for the second part
          pmp_err_d = data_pmp_err_i ;
          // The first part cannot see a PMP error in this state
          lsu_err_d = data_err_i;
          // Now we can update the address for the second part if no error
          addr_update = ~data_err_i;
          // Capture the first rdata for loads
          rdata_update = ~data_we_q;
          // Wait for second rvalid
          ls_fsm_ns = IDLE;
        end
      end

      CTX_WAIT_GNT1: begin
        if (cheri_pmode_i) begin
          addr_incr_req = 1'b0;
          data_req_o      = 1'b1;
          if (data_gnt_i) begin
            ls_fsm_ns = CTX_WAIT_GNT2;
            ctrl_update         = 1'b1;
            addr_update         = 1'b1;
           end
        end else begin
          ls_fsm_ns = IDLE;
        end
      end

      CTX_WAIT_GNT2: begin
        if (cheri_pmode_i) begin
          addr_incr_req = 1'b1;
          data_req_o      = 1'b1;
          if (data_gnt_i && (data_rvalid_i || (cap_rx_fsm_q == CRX_WAIT_RESP2))) ls_fsm_ns = IDLE;
          else if (data_gnt_i) ls_fsm_ns = CTX_WAIT_RESP;
        end else begin
          ls_fsm_ns = IDLE;
        end
      end

      CTX_WAIT_RESP: begin        // only needed if mem allows 2 active req 
        if (cheri_pmode_i) begin
          addr_incr_req = 1'b1; // stay 1 to reduce unnecessary addr toggling
          data_req_o      = 1'b0;
          if (data_rvalid_i) ls_fsm_ns = IDLE;
        end else begin
          ls_fsm_ns = IDLE;
        end
      end

      default: begin
        ls_fsm_ns = IDLE;
      end
    endcase
  end

  always_comb begin
    cap_rx_fsm_d = cap_rx_fsm_q;

    case (cap_rx_fsm_q)
      CRX_IDLE:
        if (CHERIoTEn & cheri_pmode_i & lsu_is_cap_i && (ls_fsm_ns != IDLE)) cap_rx_fsm_d = CRX_WAIT_RESP1;
      CRX_WAIT_RESP1:
        if (data_rvalid_i) cap_rx_fsm_d = CRX_WAIT_RESP2;
      CRX_WAIT_RESP2:
        if (data_rvalid_i && lsu_is_cap_i && (ls_fsm_ns != IDLE)) cap_rx_fsm_d = CRX_WAIT_RESP1;
        else if (data_rvalid_i) cap_rx_fsm_d = CRX_IDLE;
      default:;
    endcase
  end

  assign tbre_req_good  = CHERIoTEn & cheri_pmode_i & CheriTBRE & tbre_lsu_req_i & ~cpu_lsu_dec_i;

  assign resp_wait = CHERIoTEn & cheri_pmode_i & CheriTBRE & outstanding_resp_q & ~lsu_resp_valid;

  // we assume ctrl will be held till req_done asserted 
  // (once req captured in IDLE, it can be deasserted)
  logic lsu_req_done;

  assign lsu_req_done        = (lsu_go | (ls_fsm_cs != IDLE)) & (ls_fsm_ns == IDLE);

  assign lsu_req_done_o      = lsu_req_done & (~lsu_is_intl_i) & (~cur_req_is_tbre);
  assign lsu_req_done_intl_o = lsu_req_done & (lsu_is_intl_i)  & (~cur_req_is_tbre) & cheri_pmode_i;
  assign lsu_tbre_req_done_o = lsu_req_done & cur_req_is_tbre & cheri_pmode_i;

  assign lsu_addr_incr_req_o      = addr_incr_req & ~cur_req_is_tbre;
  assign lsu_tbre_addr_incr_req_o = addr_incr_req & cur_req_is_tbre;

  assign cur_req_is_tbre = CHERIoTEn & cheri_pmode_i & CheriTBRE & ((ls_fsm_cs == IDLE) ? 
                           (tbre_req_good & ~resp_wait) : req_is_tbre_q);

  // registers for FSM
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ls_fsm_cs           <= IDLE;
      handle_misaligned_q <= '0;
      pmp_err_q           <= '0;
      lsu_err_q           <= '0;
      resp_is_cap_q       <= 1'b0;
      resp_is_intl_q      <= 1'b0;
      resp_lc_clrperm_q   <= 4'h0;
      resp_is_tbre_q      <= 1'b0;
      cheri_err_q         <= 1'b0;
      cap_rx_fsm_q        <= CRX_IDLE;
      cap_lsw_err_q       <= 1'b0;
      cap_lsw_q           <= 33'h0;
      req_is_tbre_q       <= 1'b0;
      outstanding_resp_q  <= 1'b0;
    end else begin
      ls_fsm_cs           <= ls_fsm_ns;
      handle_misaligned_q <= handle_misaligned_d;
      pmp_err_q           <= pmp_err_d;
      lsu_err_q           <= lsu_err_d;
      cheri_err_q         <= cheri_err_d;

      cap_rx_fsm_q        <= cap_rx_fsm_d;

      // resp_is_cap_q aligns with responses, lsu_is_cap_i aligns with requests
      //   we use lsu_go to qualify this update
      //   - note this implies that LSU only support a outstand request at a time
      //   - new request can't be issued (go) until resp_valid
      //   - also note resp_valid is gated by (ls_fsm_cs == IDLE)
      if (lsu_go) begin
        resp_is_cap_q     <= lsu_is_cap_i;
        resp_is_intl_q    <= lsu_is_intl_i;
        resp_lc_clrperm_q <= lsu_lc_clrperm_i;
        resp_is_tbre_q    <= cur_req_is_tbre;
      end

      if (CHERIoTEn & cheri_pmode_i && (cap_rx_fsm_q == CRX_WAIT_RESP1) && data_rvalid_i && (~data_we_q))
        cap_lsw_q <= data_rdata_i;

      if (CHERIoTEn & cheri_pmode_i && (cap_rx_fsm_q == CRX_WAIT_RESP1) && data_rvalid_i)
        cap_lsw_err_q <= data_err_i;

      if (CHERIoTEn & cheri_pmode_i & lsu_go) req_is_tbre_q <= tbre_req_good;

      if (lsu_go)
        outstanding_resp_q <= 1'b1;
      else if (lsu_resp_valid)
        outstanding_resp_q <= 1'b0;

    end
  end

  /////////////
  // Outputs //
  /////////////
  logic all_resp;
  assign data_or_pmp_err    = lsu_err_q | data_err_i | pmp_err_q | (cheri_pmode_i & 
                              (cheri_err_q | (resp_is_cap_q & cap_lsw_err_q)));

  assign all_resp           = data_rvalid_i | pmp_err_q | (cheri_pmode_i & cheri_err_q);
  assign lsu_resp_valid     = all_resp & (ls_fsm_cs == IDLE) ;

  assign lsu_resp_valid_o        = lsu_resp_valid & (~cheri_pmode_i | ((~resp_is_intl_q) & (~resp_is_tbre_q))) ;
  assign lsu_resp_valid_intl_o   = lsu_resp_valid & resp_is_intl_q ;
  assign lsu_tbre_resp_valid_o   = lsu_resp_valid & resp_is_tbre_q;
  assign lsu_resp_is_wr_o        = data_we_q;

  // this goes to wb as rf_we_lsu, so needs to be gated when data needs to go back to EX
  assign lsu_rdata_valid_o  = (ls_fsm_cs == IDLE) & data_rvalid_i & ~data_or_pmp_err & ~data_we_q & 
                              (~cheri_pmode_i | ((~resp_is_intl_q) & (~resp_is_tbre_q)));

  // output to register file
  if (CHERIoTEn & ~MemCapFmt) begin : gen_memcap_rd_fmt0
    assign lsu_rdata_o = (cheri_pmode_i & resp_is_cap_q) ? cap_lsw_q : data_rdata_ext;
    assign lsu_rcap_o  = (resp_is_cap_q && data_rvalid_i && (cap_rx_fsm_q == CRX_WAIT_RESP2) && (~data_or_pmp_err)) ?
                         mem2regcap_fmt0(data_rdata_i, cap_lsw_q, resp_lc_clrperm_q) : NULL_REG_CAP;
  end else if (CHERIoTEn) begin : gen_memcap_rd_fmt1
    assign lsu_rdata_o = (cheri_pmode_i & resp_is_cap_q) ? mem2regaddr_fmt1(data_rdata_ext, cap_lsw_q, lsu_rcap_o): data_rdata_ext;
    assign lsu_rcap_o  = (resp_is_cap_q && data_rvalid_i && (cap_rx_fsm_q == CRX_WAIT_RESP2) && (~data_or_pmp_err)) ?
                         mem2regcap_fmt1(data_rdata_i, cap_lsw_q, resp_lc_clrperm_q) : NULL_REG_CAP;
  end else begin : gen_no_cap_rd
    assign lsu_rdata_o = data_rdata_ext;
    assign lsu_rcap_o  = NULL_REG_CAP;
  end
  
  
  assign lsu_tbre_raw_lsw_o = cap_lsw_q;          // "raw" memory word to tbre
  assign lsu_tbre_sel_o = cur_req_is_tbre;        // req ctrl signal mux select (to cheri_ex)

  // output data address must be word aligned
  assign data_addr_w_aligned = {data_addr[31:2], 2'b00};

  // output to data interface
  assign data_addr_o   = data_addr_w_aligned;

  assign data_wdata_o  = data_wdata;
  assign data_we_o     = lsu_we_i;
  assign data_be_o     = data_be;

  assign data_is_cap_o = lsu_is_cap_i;

  // output to ID stage: mtval + AGU for misaligned transactions
  assign addr_last_o   = addr_last_q;

  // Signal a load or store error depending on the transaction type outstanding
  assign load_err_o    = data_or_pmp_err & ~data_we_q & lsu_resp_valid & (~resp_is_intl_q) & (~resp_is_tbre_q);
  assign store_err_o   = data_or_pmp_err &  data_we_q & lsu_resp_valid & (~resp_is_intl_q) & (~resp_is_tbre_q);

  assign lsu_err_is_cheri_o  = cheri_pmode_i & cheri_err_q;     // send to controller for mcause encoding
  assign lsu_resp_err_intl_o = cheri_pmode_i & data_or_pmp_err & lsu_resp_valid & resp_is_intl_q;
  assign lsu_tbre_resp_err_o = cheri_pmode_i & data_or_pmp_err & lsu_resp_valid & resp_is_tbre_q;

  assign busy_o = (ls_fsm_cs != IDLE);
  // assign busy_tbre_o = (ls_fsm_cs != IDLE) & cur_req_is_tbre;
  assign busy_tbre_o = (ls_fsm_cs != IDLE) & cheri_pmode_i & req_is_tbre_q;

endmodule
