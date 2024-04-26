// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Sonata system, which instantiates and connects the following blocks:
// - TileLink Uncached Lightweight (TL-UL) bus.
// - Ibex top module.
// - RAM memory to contain code and data.
// - GPIO driving logic.
// - UART for serial communication.
// - Timer.
// - Debug module.
// - SPI for driving LCD screen.
module sonata_system #(
  parameter int unsigned SysClkFreq    = 50_000_000,
  parameter int unsigned GpiWidth      = 13,
  parameter int unsigned GpoWidth      = 24,
  parameter int unsigned PwmWidth      = 12,
  parameter int unsigned CheriErrWidth =  9,
  parameter SRAMInitFile               = ""
) (
  input logic                 clk_sys_i,
  input logic                 rst_sys_ni,

  input  logic [GpiWidth-1:0] gp_i,
  output logic [GpoWidth-1:0] gp_o,
  output logic [PwmWidth-1:0] pwm_o,
  input  logic                uart_rx_i,
  output logic                uart_tx_o,
  input  logic                spi_rx_i,
  output logic                spi_tx_o,
  output logic                spi_sck_o,

  input  logic tck_i,   // JTAG test clock pad
  input  logic tms_i,   // JTAG test mode select pad
  input  logic trst_ni, // JTAG test reset pad
  input  logic td_i,    // JTAG test data input pad
  output logic td_o,    // JTAG test data output pad

  output logic [CheriErrWidth-1:0] cheri_err_o,
  output logic                     cheri_en_o,

  // I2C bus 0
  input  logic                i2c0_scl_i,
  output logic                i2c0_scl_o,
  output logic                i2c0_scl_en_o,
  input  logic                i2c0_sda_i,
  output logic                i2c0_sda_o,
  output logic                i2c0_sda_en_o,

  // I2C bus 1
  input  logic                i2c1_scl_i,
  output logic                i2c1_scl_o,
  output logic                i2c1_scl_en_o,
  input  logic                i2c1_sda_i,
  output logic                i2c1_sda_o,
  output logic                i2c1_sda_en_o
);

  ///////////////////////////////////////////////
  // Signals, types and parameters for system. //
  ///////////////////////////////////////////////

  localparam int unsigned MemSize       = 128 * 1024; // 128 KiB
  localparam int unsigned SRAMAddrWidth = $clog2(MemSize);
  localparam int unsigned DebugStart    = 32'h1a110000;
  localparam int unsigned PwmCtrSize    = 8;
  localparam int unsigned BusAddrWidth  = 32;
  localparam int unsigned BusByteEnable = 4;
  localparam int unsigned BusDataWidth  = 32;
  localparam int unsigned RegAddrWidth  = 8;

  // Debug functionality is disabled.
  localparam int unsigned DbgHwBreakNum = 0;
  localparam bit          DbgTriggerEn  = 1'b0;

  typedef enum int {
    CoreD,
    DbgHost
  } bus_host_e;

  typedef enum int {
    Gpio,
    Pwm,
    Uart,
    Timer,
    Spi
  } bus_device_e;

  localparam int NrDevices = 5;
  localparam int NrHosts = 2;

  // Interrupts.
  logic timer_irq;
  logic external_irq;

  logic uart_tx_watermark_irq;
  logic uart_rx_watermark_irq;
  logic uart_tx_empty_irq;
  logic uart_rx_overflow_irq;
  logic uart_rx_frame_err_irq;
  logic uart_rx_break_err_irq;
  logic uart_rx_timeout_irq;
  logic uart_rx_parity_err_irq;

  logic i2c0_fmt_threshold_irq;
  logic i2c0_rx_threshold_irq;
  logic i2c0_acq_threshold_irq;
  logic i2c0_rx_overflow_irq;
  logic i2c0_nak_irq;
  logic i2c0_scl_interference_irq;
  logic i2c0_sda_interference_irq;
  logic i2c0_stretch_timeout_irq;
  logic i2c0_sda_unstable_irq;
  logic i2c0_cmd_complete_irq;
  logic i2c0_tx_stretch_irq;
  logic i2c0_tx_threshold_irq;
  logic i2c0_acq_full_irq;
  logic i2c0_unexp_stop_irq;
  logic i2c0_host_timeout_irq;

  logic i2c1_fmt_threshold_irq;
  logic i2c1_rx_threshold_irq;
  logic i2c1_acq_threshold_irq;
  logic i2c1_rx_overflow_irq;
  logic i2c1_nak_irq;
  logic i2c1_scl_interference_irq;
  logic i2c1_sda_interference_irq;
  logic i2c1_stretch_timeout_irq;
  logic i2c1_sda_unstable_irq;
  logic i2c1_cmd_complete_irq;
  logic i2c1_tx_stretch_irq;
  logic i2c1_tx_threshold_irq;
  logic i2c1_acq_full_irq;
  logic i2c1_unexp_stop_irq;
  logic i2c1_host_timeout_irq;

  logic [181:0] intr_vector;
  assign intr_vector = {
      143'b0, // IDs [39 +: 143]

      i2c1_host_timeout_irq, // IDs [38 +: 1]
      i2c1_unexp_stop_irq, // IDs [37 +: 1]
      i2c1_acq_full_irq, // IDs [36 +: 1]
      i2c1_tx_threshold_irq, // IDs [35 +: 1]
      i2c1_tx_stretch_irq, // IDs [34 +: 1]
      i2c1_cmd_complete_irq, // IDs [33 +: 1]
      i2c1_sda_unstable_irq, // IDs [32 +: 1]
      i2c1_stretch_timeout_irq, // IDs [31 +: 1]
      i2c1_sda_interference_irq, // IDs [30 +: 1]
      i2c1_scl_interference_irq, // IDs [29 +: 1]
      i2c1_nak_irq, // IDs [28 +: 1]
      i2c1_rx_overflow_irq, // IDs [27 +: 1]
      i2c1_acq_threshold_irq, // IDs [26 +: 1]
      i2c1_rx_threshold_irq, // IDs [25 +: 1]
      i2c1_fmt_threshold_irq, // IDs [24 +: 1]

      i2c0_host_timeout_irq, // IDs [23 +: 1]
      i2c0_unexp_stop_irq, // IDs [22 +: 1]
      i2c0_acq_full_irq, // IDs [21 +: 1]
      i2c0_tx_threshold_irq, // IDs [20 +: 1]
      i2c0_tx_stretch_irq, // IDs [19 +: 1]
      i2c0_cmd_complete_irq, // IDs [18 +: 1]
      i2c0_sda_unstable_irq, // IDs [17 +: 1]
      i2c0_stretch_timeout_irq, // IDs [16 +: 1]
      i2c0_sda_interference_irq, // IDs [15 +: 1]
      i2c0_scl_interference_irq, // IDs [14 +: 1]
      i2c0_nak_irq, // IDs [13 +: 1]
      i2c0_rx_overflow_irq, // IDs [12 +: 1]
      i2c0_acq_threshold_irq, // IDs [11 +: 1]
      i2c0_rx_threshold_irq, // IDs [10 +: 1]
      i2c0_fmt_threshold_irq, // IDs [9 +: 1]

      uart_rx_parity_err_irq, // IDs [8 +: 1]
      uart_rx_timeout_irq, // IDs [7 +: 1]
      uart_rx_break_err_irq, // IDs [6 +: 1]
      uart_rx_frame_err_irq, // IDs [5 +: 1]
      uart_rx_overflow_irq, // IDs [4 +: 1]
      uart_tx_empty_irq, // IDs [3 +: 1]
      uart_rx_watermark_irq, // IDs [2 +: 1]
      uart_tx_watermark_irq, // IDs [1 +: 1]

      1'b 0 // ID [0 +: 1] is a special case and tied to zero.
  };

  // Bus signals for host(s).
  logic                     host_req   [NrHosts];
  logic                     host_gnt   [NrHosts];
  logic [BusAddrWidth-1:0]  host_addr  [NrHosts];
  logic                     host_we    [NrHosts];
  logic [BusByteEnable-1:0] host_be    [NrHosts];
  logic [BusDataWidth-1:0]  host_wdata [NrHosts];
  logic                     host_wcap  [NrHosts];
  logic                     host_rvalid[NrHosts];
  logic [BusDataWidth-1:0]  host_rdata [NrHosts];
  logic                     host_rcap  [NrHosts];
  logic                     host_err   [NrHosts];

  logic [BusDataWidth:0] cheri_wdata; // No minus one for the tag.
  logic [BusDataWidth:0] cheri_rdata; // No minus one for the tag.

  assign host_wdata[CoreD]             = cheri_wdata[BusDataWidth-1:0];
  assign host_wcap[CoreD]              = cheri_wdata[BusDataWidth];
  assign cheri_rdata[BusDataWidth-1:0] = host_rdata[CoreD];
  assign cheri_rdata[BusDataWidth]     = host_rcap[CoreD];

  // Bus signals for devices.
  logic                     device_req   [NrDevices];
  logic [BusAddrWidth-1:0]  device_addr  [NrDevices];
  logic                     device_re    [NrDevices]; // Read enable.
  logic                     device_we    [NrDevices]; // Write enable.
  logic [BusByteEnable-1:0] device_be    [NrDevices];
  logic [BusDataWidth-1:0]  device_wdata [NrDevices];
  logic                     device_rvalid[NrDevices];
  logic [BusDataWidth-1:0]  device_rdata [NrDevices];
  logic                     device_err   [NrDevices];

  // Generate requests from read and write enables.
  assign device_req[Gpio]  = device_re[Gpio]  | device_we[Gpio];
  assign device_req[Pwm]   = device_re[Pwm]   | device_we[Pwm];
  assign device_req[Uart]  = device_re[Uart]  | device_we[Uart];
  assign device_req[Timer] = device_re[Timer] | device_we[Timer];
  assign device_req[Spi]   = device_re[Spi]   | device_we[Spi];

  // Instruction fetch signals.
  logic                    core_instr_req;
  logic                    core_instr_req_filtered;
  logic                    core_instr_gnt;
  logic                    core_instr_rvalid;
  logic [BusAddrWidth-1:0] core_instr_addr;
  logic [BusDataWidth-1:0] core_instr_rdata;
  logic                    core_instr_err;

  assign core_instr_req_filtered =
      core_instr_req & ((core_instr_addr & ~(tl_main_pkg::ADDR_MASK_SRAM)) == tl_main_pkg::ADDR_SPACE_SRAM);

  // Reset signals
  // Internally generated resets cause IMPERFECTSCH warnings
  /* verilator lint_off IMPERFECTSCH */
  logic rst_core_n;
  logic ndmreset_req;
  /* verilator lint_on IMPERFECTSCH */

  // Tie-off unused error signals.
  assign device_err[Gpio] = 1'b0;
  assign device_err[Pwm]  = 1'b0;
  assign device_err[Uart] = 1'b0;
  assign device_err[Spi]  = 1'b0;

  //////////////////////////////////////////////
  // Instantiate TL-UL crossbar and adapters. //
  //////////////////////////////////////////////

  // Host interfaces.
  tlul_pkg::tl_h2d_t tl_ibex_ins_h2d;
  tlul_pkg::tl_d2h_t tl_ibex_ins_d2h;

  tlul_pkg::tl_h2d_t tl_ibex_lsu_h2d_d;
  tlul_pkg::tl_d2h_t tl_ibex_lsu_d2h_d;
  tlul_pkg::tl_h2d_t tl_ibex_lsu_h2d_q;
  tlul_pkg::tl_d2h_t tl_ibex_lsu_d2h_q;

  tlul_pkg::tl_h2d_t tl_dbg_host_h2d_d;
  tlul_pkg::tl_d2h_t tl_dbg_host_d2h_d;
  tlul_pkg::tl_h2d_t tl_dbg_host_h2d_q;
  tlul_pkg::tl_d2h_t tl_dbg_host_d2h_q;

  // Device interfaces.
  tlul_pkg::tl_h2d_t tl_sram_h2d_d;
  tlul_pkg::tl_d2h_t tl_sram_d2h_d;
  tlul_pkg::tl_h2d_t tl_sram_h2d_q;
  tlul_pkg::tl_d2h_t tl_sram_d2h_q;
  tlul_pkg::tl_h2d_t tl_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_uart_h2d;
  tlul_pkg::tl_d2h_t tl_uart_d2h;
  tlul_pkg::tl_h2d_t tl_timer_h2d;
  tlul_pkg::tl_d2h_t tl_timer_d2h;
  tlul_pkg::tl_h2d_t tl_pwm_h2d;
  tlul_pkg::tl_d2h_t tl_pwm_d2h;
  tlul_pkg::tl_h2d_t tl_i2c0_h2d;
  tlul_pkg::tl_d2h_t tl_i2c0_d2h;
  tlul_pkg::tl_h2d_t tl_i2c1_h2d;
  tlul_pkg::tl_d2h_t tl_i2c1_d2h;
  tlul_pkg::tl_h2d_t tl_spi_h2d;
  tlul_pkg::tl_d2h_t tl_spi_d2h;
  tlul_pkg::tl_h2d_t tl_rv_plic_h2d;
  tlul_pkg::tl_d2h_t tl_rv_plic_d2h;

  xbar_main xbar (
    .clk_sys_i   (clk_sys_i),
    .rst_sys_ni  (rst_sys_ni),

    // Host interfaces.
    .tl_ibex_lsu_i(tl_ibex_lsu_h2d_q),
    .tl_ibex_lsu_o(tl_ibex_lsu_d2h_q),
    .tl_dbg_host_i(tl_dbg_host_h2d_q),
    .tl_dbg_host_o(tl_dbg_host_d2h_q),

    // Device interfaces.
    .tl_sram_o    (tl_sram_h2d_d),
    .tl_sram_i    (tl_sram_d2h_d),
    .tl_gpio_o    (tl_gpio_h2d),
    .tl_gpio_i    (tl_gpio_d2h),
    .tl_uart_o    (tl_uart_h2d),
    .tl_uart_i    (tl_uart_d2h),
    .tl_timer_o   (tl_timer_h2d),
    .tl_timer_i   (tl_timer_d2h),
    .tl_pwm_o     (tl_pwm_h2d),
    .tl_pwm_i     (tl_pwm_d2h),
    .tl_i2c0_o    (tl_i2c0_h2d),
    .tl_i2c0_i    (tl_i2c0_d2h),
    .tl_i2c1_o    (tl_i2c1_h2d),
    .tl_i2c1_i    (tl_i2c1_d2h),
    .tl_spi_o     (tl_spi_h2d),
    .tl_spi_i     (tl_spi_d2h),
    .tl_rv_plic_o (tl_rv_plic_h2d),
    .tl_rv_plic_i (tl_rv_plic_d2h),

    .scanmode_i(prim_mubi_pkg::MuBi4False)
  );

  // TL-UL host adapter(s).

  tlul_adapter_host ibex_ins_host_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .req_i       (core_instr_req_filtered),
    .gnt_o       (core_instr_gnt),
    .addr_i      (core_instr_addr),
    .we_i        ('0),
    .wdata_i     ('0),
    .wdata_cap_i ('0),
    .wdata_intg_i('0),
    .be_i        ('0),
    .instr_type_i(prim_mubi_pkg::MuBi4True),

    .valid_o     (core_instr_rvalid),
    .rdata_o     (core_instr_rdata),
    .rdata_cap_o (), // Instructions should not have capability tag set.
    .rdata_intg_o(),
    .err_o       (core_instr_err),
    .intg_err_o  (),

    .tl_o(tl_ibex_ins_h2d),
    .tl_i(tl_ibex_ins_d2h)
  );

  tlul_adapter_host ibex_lsu_host_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .req_i       (host_req[CoreD]),
    .gnt_o       (host_gnt[CoreD]),
    .addr_i      (host_addr[CoreD]),
    .we_i        (host_we[CoreD]),
    .wdata_i     (host_wdata[CoreD]),
    .wdata_cap_i (host_wcap[CoreD]),
    .wdata_intg_i('0),
    .be_i        (host_be[CoreD]),
    .instr_type_i(prim_mubi_pkg::MuBi4False),

    .valid_o     (host_rvalid[CoreD]),
    .rdata_o     (host_rdata[CoreD]),
    .rdata_cap_o (host_rcap[CoreD]),
    .rdata_intg_o(),
    .err_o       (host_err[CoreD]),
    .intg_err_o  (),

    .tl_o(tl_ibex_lsu_h2d_d),
    .tl_i(tl_ibex_lsu_d2h_d)
  );

  tlul_adapter_host dbg_host_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .req_i       (host_req[DbgHost]),
    .gnt_o       (host_gnt[DbgHost]),
    .addr_i      (host_addr[DbgHost]),
    .we_i        (host_we[DbgHost]),
    .wdata_i     (host_wdata[DbgHost]),
    .wdata_cap_i (host_wcap[DbgHost]),
    .wdata_intg_i('0),
    .be_i        (host_be[DbgHost]),
    .instr_type_i(prim_mubi_pkg::MuBi4False),

    .valid_o     (host_rvalid[DbgHost]),
    .rdata_o     (host_rdata[DbgHost]),
    .rdata_cap_o (host_rcap[DbgHost]),
    .rdata_intg_o(),
    .err_o       (host_err[DbgHost]),
    .intg_err_o  (),

    .tl_o(tl_dbg_host_h2d_d),
    .tl_i(tl_dbg_host_d2h_d)
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_ibex_lsu_fifo (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .tl_h_i(tl_ibex_lsu_h2d_d),
    .tl_h_o(tl_ibex_lsu_d2h_d),
    .tl_d_o(tl_ibex_lsu_h2d_q),
    .tl_d_i(tl_ibex_lsu_d2h_q),

    .spare_req_i(1'b0),
    .spare_req_o(),
    .spare_rsp_i(1'b0),
    .spare_rsp_o()
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_dbg_host_fifo (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .tl_h_i(tl_dbg_host_h2d_d),
    .tl_h_o(tl_dbg_host_d2h_d),
    .tl_d_o(tl_dbg_host_h2d_q),
    .tl_d_i(tl_dbg_host_d2h_q),

    .spare_req_i(1'b0),
    .spare_req_o(),
    .spare_rsp_i(1'b0),
    .spare_rsp_o()
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_sram_fifo (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .tl_h_i(tl_sram_h2d_d),
    .tl_h_o(tl_sram_d2h_d),
    .tl_d_o(tl_sram_h2d_q),
    .tl_d_i(tl_sram_d2h_q),

    .spare_req_i(1'b0),
    .spare_req_o(),
    .spare_rsp_i(1'b0),
    .spare_rsp_o()
  );

  sram #(
    .AddrWidth ( SRAMAddrWidth ),
    .InitFile  ( SRAMInitFile  )
  ) u_sram_top (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .tl_a_i (tl_sram_h2d_q),
    .tl_a_o (tl_sram_d2h_q),
    .tl_b_i (tl_ibex_ins_h2d),
    .tl_b_o (tl_ibex_ins_d2h)
  );

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) gpio_device_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    // TL-UL interface.
    .tl_i(tl_gpio_h2d),
    .tl_o(tl_gpio_d2h),

    // Control interface.
    .en_ifetch_i (prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    // Register interface.
    .re_o   (device_re[Gpio]),
    .we_o   (device_we[Gpio]),
    .addr_o (device_addr[Gpio][RegAddrWidth-1:0]),
    .wdata_o(device_wdata[Gpio]),
    .be_o   (device_be[Gpio]),
    .busy_i ('0),
    .rdata_i(device_rdata[Gpio]),
    .error_i(device_err[Gpio])
  );

  // Tie off upper bits of address.
  assign device_addr[Gpio][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) pwm_device_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    // TL-UL interface.
    .tl_i(tl_pwm_h2d),
    .tl_o(tl_pwm_d2h),

    // Control interface.
    .en_ifetch_i (prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    // Register interface.
    .re_o   (device_re[Pwm]),
    .we_o   (device_we[Pwm]),
    .addr_o (device_addr[Pwm][RegAddrWidth-1:0]),
    .wdata_o(device_wdata[Pwm]),
    .be_o   (device_be[Pwm]),
    .busy_i ('0),
    .rdata_i(device_rdata[Pwm]),
    .error_i(device_err[Pwm])
  );

  // Tie off upper bits of address.
  assign device_addr[Pwm][BusAddrWidth-1:RegAddrWidth] = '0;

  // Tie off upper bits of address.
  assign device_addr[Uart][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) timer_device_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    // TL-UL interface.
    .tl_i(tl_timer_h2d),
    .tl_o(tl_timer_d2h),

    // Control interface.
    .en_ifetch_i (prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    // Register interface.
    .re_o   (device_re[Timer]),
    .we_o   (device_we[Timer]),
    .addr_o (device_addr[Timer][RegAddrWidth-1:0]),
    .wdata_o(device_wdata[Timer]),
    .be_o   (device_be[Timer]),
    .busy_i ('0),
    .rdata_i(device_rdata[Timer]),
    .error_i(device_err[Timer])
  );

  // Tie off upper bits of address.
  assign device_addr[Timer][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) spi_device_adapter (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    // TL-UL interface.
    .tl_i(tl_spi_h2d),
    .tl_o(tl_spi_d2h),

    // Control interface.
    .en_ifetch_i (prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    // Register interface.
    .re_o   (device_re[Spi]),
    .we_o   (device_we[Spi]),
    .addr_o (device_addr[Spi][RegAddrWidth-1:0]),
    .wdata_o(device_wdata[Spi]),
    .be_o   (device_be[Spi]),
    .busy_i ('0),
    .rdata_i(device_rdata[Spi]),
    .error_i(device_err[Spi])
  );

  // Tie off upper bits of address.
  assign device_addr[Spi][BusAddrWidth-1:RegAddrWidth] = '0;

  ///////////////////////////////////////////////
  // Core and hardware IP block instantiation. //
  ///////////////////////////////////////////////

  logic cheri_en;

  assign cheri_en = 1'b1;
  assign cheri_en_o = cheri_en;
  assign rst_core_n = rst_sys_ni & ~ndmreset_req & ~host_req[DbgHost];

  ibexc_top_tracing #(
    .DmHaltAddr      ( DebugStart + dm::HaltAddress[31:0]      ),
    .DmExceptionAddr ( DebugStart + dm::ExceptionAddress[31:0] ),
    .DbgTriggerEn    ( DbgTriggerEn                            ),
    .DbgHwBreakNum   ( DbgHwBreakNum                           ),
    .MHPMCounterNum  ( 13                                      ),
    .RV32B           ( ibex_pkg::RV32BFull                     )
  ) u_top_tracing (
    .clk_i (clk_sys_i),
    .rst_ni(rst_core_n),

    .test_en_i  (1'b0),
    .scan_rst_ni(1'b1),
    .ram_cfg_i  (10'b0),

    .cheri_pmode_i (cheri_en),
    .cheri_tsafe_en_i (1'b0), // TODO enable temporal safety.
    .cheri_err_o (cheri_err_o),

    .hart_id_i(32'b0),
    // First instruction executed is at 0x0 + 0x80.
    .boot_addr_i(32'h00100000),

    .instr_req_o       (core_instr_req),
    .instr_gnt_i       (core_instr_gnt),
    .instr_rvalid_i    (core_instr_rvalid),
    .instr_addr_o      (core_instr_addr),
    .instr_rdata_i     (core_instr_rdata),
    .instr_rdata_intg_i('0),
    .instr_err_i       (core_instr_err),

    .data_req_o       (host_req[CoreD]),
    .data_is_cap_o    (), // TODO connect this to memory when CHERI is enabled.
    .data_gnt_i       (host_gnt[CoreD]),
    .data_rvalid_i    (host_rvalid[CoreD]),
    .data_we_o        (host_we[CoreD]),
    .data_be_o        (host_be[CoreD]),
    .data_addr_o      (host_addr[CoreD]),
    .data_wdata_o     (cheri_wdata),
    .data_wdata_intg_o(),
    .data_rdata_i     (cheri_rdata),
    .data_rdata_intg_i('0),
    .data_err_i       (host_err[CoreD]),

    // TODO fill this in once revocation is enabled.
    .tsmap_cs_o   (),
    .tsmap_addr_o (),
    .tsmap_rdata_i(32'b0),

    // TODO fill this in.
    .mmreg_corein_i  (128'b0),
    .mmreg_coreout_o (),
    .cheri_fatal_err_o(),

    .irq_software_i(1'b0),
    .irq_timer_i   (timer_irq),
    .irq_external_i(external_irq),
    .irq_fast_i    (15'b0),
    .irq_nm_i      (1'b0),

    .scramble_key_valid_i('0),
    .scramble_key_i      ('0),
    .scramble_nonce_i    ('0),
    .scramble_req_o      (),

    .debug_req_i        (),
    .crash_dump_o       (),
    .double_fault_seen_o(),

    .fetch_enable_i        ('1),
    .alert_minor_o         (),
    .alert_major_internal_o(),
    .alert_major_bus_o     (),
    .core_sleep_o          ()
  );

  gpio #(
    .GpiWidth ( GpiWidth ),
    .GpoWidth ( GpoWidth )
  ) u_gpio (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Gpio]),
    .device_addr_i  (device_addr[Gpio]),
    .device_we_i    (device_we[Gpio]),
    .device_be_i    (device_be[Gpio]),
    .device_wdata_i (device_wdata[Gpio]),
    .device_rvalid_o(device_rvalid[Gpio]),
    .device_rdata_o (device_rdata[Gpio]),

    .gp_i,
    .gp_o
  );

  i2c u_i2c0(
      .clk_i                    (clk_sys_i),
      .rst_ni                   (rst_sys_ni),
      .ram_cfg_i                ('b0),

      // Bus Interface
      .tl_i                     (tl_i2c0_h2d),
      .tl_o                     (tl_i2c0_d2h),

      // Generic IO
      .cio_scl_i                (i2c0_scl_i),
      .cio_scl_o                (i2c0_scl_o),
      .cio_scl_en_o             (i2c0_scl_en_o),
      .cio_sda_i                (i2c0_sda_i),
      .cio_sda_o                (i2c0_sda_o),
      .cio_sda_en_o             (i2c0_sda_en_o),

      // Interrupts
      .intr_fmt_threshold_o     (i2c0_fmt_threshold_irq),
      .intr_rx_threshold_o      (i2c0_rx_threshold_irq),
      .intr_acq_threshold_o     (i2c0_acq_threshold_irq),
      .intr_rx_overflow_o       (i2c0_rx_overflow_irq),
      .intr_nak_o               (i2c0_nak_irq),
      .intr_scl_interference_o  (i2c0_scl_interference_irq),
      .intr_sda_interference_o  (i2c0_sda_interference_irq),
      .intr_stretch_timeout_o   (i2c0_stretch_timeout_irq),
      .intr_sda_unstable_o      (i2c0_sda_unstable_irq),
      .intr_cmd_complete_o      (i2c0_cmd_complete_irq),
      .intr_tx_stretch_o        (i2c0_tx_stretch_irq),
      .intr_tx_threshold_o      (i2c0_tx_threshold_irq),
      .intr_acq_full_o          (i2c0_acq_full_irq),
      .intr_unexp_stop_o        (i2c0_unexp_stop_irq),
      .intr_host_timeout_o      (i2c0_host_timeout_irq)
  );

  i2c u_i2c1(
      .clk_i                    (clk_sys_i),
      .rst_ni                   (rst_sys_ni),
      .ram_cfg_i                ('b0),

      // Bus Interface
      .tl_i                     (tl_i2c1_h2d),
      .tl_o                     (tl_i2c1_d2h),

      // Generic IO
      .cio_scl_i                (i2c1_scl_i),
      .cio_scl_o                (i2c1_scl_o),
      .cio_scl_en_o             (i2c1_scl_en_o),
      .cio_sda_i                (i2c1_sda_i),
      .cio_sda_o                (i2c1_sda_o),
      .cio_sda_en_o             (i2c1_sda_en_o),

      // Interrupts
      .intr_fmt_threshold_o     (i2c1_fmt_threshold_irq),
      .intr_rx_threshold_o      (i2c1_rx_threshold_irq),
      .intr_acq_threshold_o     (i2c1_acq_threshold_irq),
      .intr_rx_overflow_o       (i2c1_rx_overflow_irq),
      .intr_nak_o               (i2c1_nak_irq),
      .intr_scl_interference_o  (i2c1_scl_interference_irq),
      .intr_sda_interference_o  (i2c1_sda_interference_irq),
      .intr_stretch_timeout_o   (i2c1_stretch_timeout_irq),
      .intr_sda_unstable_o      (i2c1_sda_unstable_irq),
      .intr_cmd_complete_o      (i2c1_cmd_complete_irq),
      .intr_tx_stretch_o        (i2c1_tx_stretch_irq),
      .intr_tx_threshold_o      (i2c1_tx_threshold_irq),
      .intr_acq_full_o          (i2c1_acq_full_irq),
      .intr_unexp_stop_o        (i2c1_unexp_stop_irq),
      .intr_host_timeout_o      (i2c1_host_timeout_irq)
  );

  pwm_wrapper #(
    .PwmWidth   ( PwmWidth   ),
    .PwmCtrSize ( PwmCtrSize )
  ) u_pwm (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Pwm]),
    .device_addr_i  (device_addr[Pwm]),
    .device_we_i    (device_we[Pwm]),
    .device_be_i    (device_be[Pwm]),
    .device_wdata_i (device_wdata[Pwm]),
    .device_rvalid_o(device_rvalid[Pwm]),
    .device_rdata_o (device_rdata[Pwm]),

    .pwm_o
  );

  uart u_uart (
      .clk_i       (clk_sys_i  ),
      .rst_ni      (rst_sys_ni ),

      .cio_rx_i    (uart_rx_i  ),
      .cio_tx_o    (uart_tx_o  ),
      .cio_tx_en_o (           ),

      // Inter-module signals
      .tl_i        (tl_uart_h2d),
      .tl_o        (tl_uart_d2h),

      // Interrupt
      .intr_tx_watermark_o  (uart_tx_watermark_irq),
      .intr_rx_watermark_o  (uart_rx_watermark_irq),
      .intr_tx_empty_o      (uart_tx_empty_irq),
      .intr_rx_overflow_o   (uart_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart_rx_break_err_irq),
      .intr_rx_timeout_o    (uart_rx_timeout_irq),
      .intr_rx_parity_err_o (uart_rx_parity_err_irq)
  );

  spi_top #(
    .ClockFrequency (SysClkFreq),
    .CPOL           ( 0        ),
    .CPHA           ( 1        )
  ) u_spi (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .device_req_i   (device_req[Spi]),
    .device_addr_i  (device_addr[Spi]),
    .device_we_i    (device_we[Spi]),
    .device_be_i    (device_be[Spi]),
    .device_wdata_i (device_wdata[Spi]),
    .device_rvalid_o(device_rvalid[Spi]),
    .device_rdata_o (device_rdata[Spi]),

    .spi_rx_i(spi_rx_i),  // Data received from SPI device.
    .spi_tx_o(spi_tx_o),  // Data transmitted to SPI device.
    .sck_o   (spi_sck_o), // Serial clock pin.

    .byte_data_o()
  );

  timer #(
    .DataWidth    ( BusDataWidth ),
    .AddressWidth ( BusAddrWidth )
  ) u_timer (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),

    .timer_req_i   (device_req[Timer]),
    .timer_we_i    (device_we[Timer]),
    .timer_be_i    (device_be[Timer]),
    .timer_addr_i  (device_addr[Timer]),
    .timer_wdata_i (device_wdata[Timer]),
    .timer_rvalid_o(device_rvalid[Timer]),
    .timer_rdata_o (device_rdata[Timer]),
    .timer_err_o   (device_err[Timer]),
    .timer_intr_o  (timer_irq)
  );

  rv_plic u_rv_plic (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .irq_o    (external_irq),
    .irq_id_o (),
    .tl_i     (tl_rv_plic_h2d),
    .tl_o     (tl_rv_plic_d2h),

    .intr_src_i (intr_vector)
  );

  dm_top #(
    .NrHarts      ( 1                              ),
    .IdcodeValue  ( jtag_id_pkg::RV_DM_JTAG_IDCODE )
  ) u_dm_top (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),
    .testmode_i   (1'b0),
    .ndmreset_o   (ndmreset_req),
    .dmactive_o   (),
    .debug_req_o  (), // TODO connect to debug_req_i
    .unavailable_i(1'b0),

    // TODO: Bus device with debug memory (for execution-based debug).
    .device_req_i  (0),
    .device_we_i   (0),
    .device_addr_i (0),
    .device_be_i   (0),
    .device_wdata_i(0),
    .device_rdata_o(),

    // Bus host (for system bus accesses, SBA).
    .host_req_o    (host_req[DbgHost]),
    .host_add_o    (host_addr[DbgHost]),
    .host_we_o     (host_we[DbgHost]),
    .host_wdata_o  (host_wdata[DbgHost]),
    .host_be_o     (host_be[DbgHost]),
    .host_gnt_i    (host_gnt[DbgHost]),
    .host_r_valid_i(host_rvalid[DbgHost]),
    .host_r_rdata_i(host_rdata[DbgHost]),

    .tck_i,
    .tms_i,
    .trst_ni,
    .td_i,
    .td_o
  );

  `ifdef VERILATOR
    export "DPI-C" function mhpmcounter_get;

    function automatic longint unsigned mhpmcounter_get(int index);
      return u_top_tracing.u_ibex_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
    endfunction
  `endif

  for (genvar i = 0; i < NrDevices; i++) begin : gen_unused_device
    logic _unused_rvalid;
    assign _unused_rvalid = device_rvalid[i];
  end : gen_unused_device
endmodule
