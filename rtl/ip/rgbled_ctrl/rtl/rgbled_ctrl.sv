// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module rgbled_ctrl import rgbled_ctrl_reg_pkg::*; #(
  parameter int unsigned CycleTime = 31,
  parameter int unsigned StartupWaitCycles = 5000,
  parameter int unsigned StartupRepeats = 2
) (
  input clk_i,
  input rst_ni,

  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  output logic rgbled_dout_o
);

  rgbled_ctrl_reg2hw_t reg2hw;
  rgbled_ctrl_hw2reg_t hw2reg;

  logic go, drv_idle, idle, off;
  logic [23:0] grb_data;
  logic grb_data_valid, grb_data_last, grb_data_ack;
  logic grb_data_sel_q, grb_data_sel_d;

  // TODO: This should probably be a generate loop!

  // grb0/grb1 are the flops that hold the two LED colour values. They are passed to the ws281x
  // driver when a setrgb command is received from software. grb (green/red/blue) is the order the
  // ws281x chips expect the colours in.
  logic [23:0] grb0_q, grb0_d, grb1_q, grb1_d;
  logic        grb0_en, grb1_en;

  // Only write a new value if idle. A write can be triggered by a direct write from software or
  // indirectly through an off command.
  assign grb0_en = idle & (reg2hw.rgbled0.b.qe | off);
  // Write 0 for the off command otherwise write the software supplied values.
  assign grb0_d  = reg2hw.rgbled0.b.qe ? {reg2hw.rgbled0.g.q, reg2hw.rgbled0.r.q, reg2hw.rgbled0.b.q} :
                                         '0;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb0_q <= '0;
    end else begin
      if (grb0_en) begin
        grb0_q <= grb0_d;
      end
    end
  end

  // Only write a new value if idle. A write can be triggered by a direct write from software or
  // indirectly through an off command.
  assign grb1_en = idle & (reg2hw.rgbled1.b.qe | off);
  // Write 0 for the off command otherwise write the software supplied values.
  assign grb1_d  = reg2hw.rgbled1.b.qe ? {reg2hw.rgbled1.g.q, reg2hw.rgbled1.r.q, reg2hw.rgbled1.b.q} :
                                         '0;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb1_q <= '0;
    end else begin
      if (grb1_en) begin
        grb1_q <= grb1_d;
      end
    end
  end

  // We always have valid data ready for the ws281x driver.
  assign grb_data_valid = 1'b1;
  // When issuing an off command force 0 for data in as the off command will start the driver the
  // cycle before the grb flops are upated. Otherwise choose grb 0 or 1 depending on which needs to
  // be presented to the driver next.
  assign grb_data       = off            ? '0     :
                          grb_data_sel_q ? grb0_q :
                                           grb1_q;

  // Indicate last data when we're pasasing grb1
  assign grb_data_last  = grb_data_sel_q;

  // Flip the GRB flop select when data has been acknowledged by the driver
  assign grb_data_sel_d = grb_data_ack ? ~grb_data_sel_q : grb_data_sel_q;

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      grb_data_sel_q   <= 1'b0;
    end else begin
      grb_data_sel_q   <= grb_data_sel_d;
    end
  end

  // The RGB LEDs have a habit of turning on following reset. The original version of this
  // controller cleared the LEDs (write 0 for all RGB values for both LCDs) immediately on reset but
  // this was not sufficient.  The controller waits for a number of cycles (StartupWaitCycles)
  // following reset before it writes the clear and then repeats this (StartupRepeats times) with
  // the same wait interval between each repeat. If there is any write to the controller during this
  // startup period the startup is aborted (on the assumption the software is about to set the
  // LEDs). It would be possible to report the controller is not idle during the startup period
  // but this may cause software that immediately begins using the RGB LED to needlessly wait for
  // the startup period to end.

  localparam int StartupWaitCyclesW = $clog2(StartupWaitCycles + 1);
  logic [StartupWaitCyclesW-1:0] wait_counter_q, wait_counter_d;

  localparam int StartupRepeatsW = $clog2(StartupRepeats + 1);
  logic [StartupRepeatsW-1:0] startup_counter_q, startup_counter_d;
  logic startup_go, startup_lockout;

  typedef enum logic [1:0] {
    STARTUP_WAIT_START = 2'b00,
    STARTUP_DO_GO = 2'b01,
    STARTUP_WAIT_IDLE = 2'b10,
    STARTUP_DONE = 2'b11
  } startup_state_e;

  startup_state_e startup_state_d, startup_state_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      startup_state_q <= STARTUP_WAIT_START;
      wait_counter_q  <= '0;
      startup_counter_q <= '0;
    end else begin
      startup_state_q   <= startup_state_d;
      wait_counter_q    <= wait_counter_d;
      startup_counter_q <= startup_counter_d;
    end
  end

  always_comb begin
    wait_counter_d    = wait_counter_q;
    startup_state_d   = startup_state_q;
    startup_counter_d = startup_counter_q;
    startup_go        = 1'b0;
    startup_lockout   = 1'b0;

    case (startup_state_q)
      STARTUP_WAIT_START: begin
        // Waiting StartupWaitCycles before writing 0s for all LED values
        if (wait_counter_q < StartupWaitCyclesW'(StartupWaitCycles)) begin
          wait_counter_d = wait_counter_q + 1'b1;
        end else begin
          wait_counter_d  = '0;
          startup_state_d = STARTUP_DO_GO;
        end
      end
      STARTUP_DO_GO: begin
        if (drv_idle) begin
          // When idle begin writing 0s to the driver.
          startup_go      = 1'b1;
          // Stop softare writes from doing anything this cycle.
          startup_lockout = 1'b1;

          if (startup_counter_q < StartupRepeatsW'(StartupRepeats)) begin
            // Wait for the driver to be idle after the go if we have more startup repeats to do.
            startup_state_d   = STARTUP_WAIT_IDLE;
            startup_counter_d = startup_counter_q + 1'b1;
          end else begin
            // Otherwise we're done
            startup_state_d = STARTUP_DONE;
          end
        end
      end
      STARTUP_WAIT_IDLE: begin
        if (drv_idle) begin
          // Wait for driver to be idle before going back to a new startup wait
          startup_state_d = STARTUP_WAIT_START;
        end
      end
      default: ;
    endcase

    // Any write from software aborts the startup proceedure unless we're stopping software
    // commands.
    if (!startup_lockout && (reg2hw.ctrl.off.qe | reg2hw.ctrl.setrgb.qe |
        reg2hw.rgbled1.b.qe | reg2hw.rgbled0.b.qe)) begin
      startup_state_d = STARTUP_DONE;
    end
  end

  rgbled_ctrl_reg_top u_rgbled_ctrl_reg_top (
    .clk_i,
    .rst_ni,
    .tl_i,
    .tl_o,
    .reg2hw,
    .hw2reg,
    .intg_err_o()
  );

  assign off = (reg2hw.ctrl.off.qe    & reg2hw.ctrl.off.q)    | startup_go;
  assign go  = (reg2hw.ctrl.setrgb.qe & reg2hw.ctrl.setrgb.q) | off;

  ws281x_drv #(
    .CycleTime(CycleTime)
  ) u_ws281x_drv (
    .clk_i,
    .rst_ni,

    .go_i  (go),
    .idle_o(drv_idle),

    .data_i      (grb_data),
    .data_valid_i(grb_data_valid),
    .data_last_i (grb_data_last),
    .data_ack_o  (grb_data_ack),

    .ws281x_dout_o(rgbled_dout_o)
  );

  // Indicate not idle when we're just about to do a startup clear to avoid a conflict with
  // a software command.
  assign idle = drv_idle & ~startup_lockout;

  assign hw2reg.status.d = idle;
endmodule
