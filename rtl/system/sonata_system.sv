// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Sonata system, which instantiates and connects the following blocks:
// - TileLink Uncached Lightweight (TL-UL) bus.
// - Ibex top module.
// - RAM memory to contain code and data.
// - GPIO driving logic.
// - Two I2C controllers for serial peripherals.
// - SPI for driving LCD screen, flash and Ethernet.
// - Timer.
// - Two UARTs for serial communication.
// - USB device.
// - Debug module.
module sonata_system #(
  parameter int unsigned GpiWidth      = 13,
  parameter int unsigned GpoWidth      = 24,
  parameter int unsigned RpiGpiWidth   = 11,
  parameter int unsigned ArdGpiWidth   = 10,
  parameter int unsigned PmodGpiWidth  = 16,
  parameter int unsigned WordWidth     = 32,
  parameter int unsigned PwmWidth      = 12,
  parameter int unsigned CheriErrWidth =  9,
  parameter SRAMInitFile               = "",
  parameter int unsigned SysClkFreq    = 30_000_000
) (
  // Main system clock and reset
  input logic                      clk_sys_i,
  input logic                      rst_sys_ni,

  // USB device clock and reset
  input logic                      clk_usb_i,
  input logic                      rst_usb_ni,

  // General purpose input and output
  input  logic [GpiWidth-1:0]      gp_i,
  output logic [GpoWidth-1:0]      gp_o,
  output logic [PwmWidth-1:0]      pwm_o,
  input  logic [RpiGpiWidth-1:0]   rp_gp_i,
  output logic [WordWidth-1:0]     rp_gp_o,
  input  logic [ArdGpiWidth-1:0]   ard_gp_i,
  output logic [WordWidth-1:0]     ard_gp_o,
  input  logic [PmodGpiWidth-1:0]  pmod_gp_i,
  output logic [WordWidth-1:0]     pmod_gp_o,

  // UART 0
  input  logic                     uart0_rx_i,
  output logic                     uart0_tx_o,

  // UART 1
  input  logic                     uart1_rx_i,
  output logic                     uart1_tx_o,

  // UART 2 Raspberry Pi HAT
  input  logic                     uart2_rx_i,
  output logic                     uart2_tx_o,

  // UART 3 mikroBUS Click
  input  logic                     uart3_rx_i,
  output logic                     uart3_tx_o,

  // UART 4 RS-232
  input  logic                     uart4_rx_i,
  output logic                     uart4_tx_o,

  // SPI flash
  input  logic                     spi_flash_rx_i,
  output logic                     spi_flash_tx_o,
  output logic                     spi_flash_sck_o,

  // SPI for LCD screen
  input  logic                     spi_lcd_rx_i,
  output logic                     spi_lcd_tx_o,
  output logic                     spi_lcd_sck_o,

  // SPI for ethernet
  input  logic                     spi_eth_rx_i,
  output logic                     spi_eth_tx_o,
  output logic                     spi_eth_sck_o,
  input  logic                     spi_eth_irq_ni, // Interrupt from Ethernet MAC

  // SPI0 on the R-Pi header
  input  logic                     spi_rp0_rx_i,
  output logic                     spi_rp0_tx_o,
  output logic                     spi_rp0_sck_o,

  // SPI1 on the R-Pi header
  input  logic                     spi_rp1_rx_i,
  output logic                     spi_rp1_tx_o,
  output logic                     spi_rp1_sck_o,

  // SPI on Arduino shield
  input  logic                     spi_ard_rx_i,
  output logic                     spi_ard_tx_o,
  output logic                     spi_ard_sck_o,

  // SPI on mikroBUS Click
  input  logic                     spi_mkr_rx_i,
  output logic                     spi_mkr_tx_o,
  output logic                     spi_mkr_sck_o,

  // User JTAG
  input  logic                     tck_i,   // JTAG test clock pad
  input  logic                     tms_i,   // JTAG test mode select pad
  input  logic                     trst_ni, // JTAG test reset pad
  input  logic                     td_i,    // JTAG test data input pad
  output logic                     td_o,    // JTAG test data output pad

  // CHERI signals
  input  logic                     cheri_en_i, // TODO: Development assistance.
  output logic [CheriErrWidth-1:0] cheri_err_o,
  output logic                     cheri_en_o,

  // I2C bus 0
  input  logic                     i2c0_scl_i,
  output logic                     i2c0_scl_o,
  output logic                     i2c0_scl_en_o,
  input  logic                     i2c0_sda_i,
  output logic                     i2c0_sda_o,
  output logic                     i2c0_sda_en_o,

  // I2C bus 1
  input  logic                     i2c1_scl_i,
  output logic                     i2c1_scl_o,
  output logic                     i2c1_scl_en_o,
  input  logic                     i2c1_sda_i,
  output logic                     i2c1_sda_o,
  output logic                     i2c1_sda_en_o,

  // Reception from USB host via transceiver
  input  logic                     usb_dp_i,
  input  logic                     usb_dn_i,
  input  logic                     usb_rx_d_i,

  // Transmission to USB host via transceiver
  output logic                     usb_dp_o,
  output logic                     usb_dp_en_o,
  output logic                     usb_dn_o,
  output logic                     usb_dn_en_o,

  // Configuration and control of USB transceiver
  input  logic                     usb_sense_i,
  output logic                     usb_dp_pullup_o,
  output logic                     usb_dn_pullup_o,
  output logic                     usb_rx_enable_o,

  output logic                     rgbled_dout_o
);

  ///////////////////////////////////////////////
  // Signals, types and parameters for system. //
  ///////////////////////////////////////////////

  localparam int unsigned MemSize       = 256 * 1024; // 256 KiB
  localparam int unsigned SRAMAddrWidth = $clog2(MemSize);
  localparam int unsigned DebugStart    = 32'h1a110000;
  localparam int unsigned PwmCtrSize    = 8;
  localparam int unsigned BusAddrWidth  = 32;
  localparam int unsigned BusByteEnable = 4;
  localparam int unsigned BusDataWidth  = 32;
  localparam int unsigned RegAddrWidth  = 8;
  localparam int unsigned TRegAddrWidth = 16;  // Timer uses more address bits.

  // The number of data bits controlled by each mask bit; since the CPU requires
  // only byte level access, explicitly grouping the data bits makes the inferred
  // BRAM implementations in FPGA much more efficient.
  localparam int unsigned DataBitsPerMask = BusDataWidth / BusByteEnable;

  // Debug functionality is disabled.
  localparam int unsigned DbgHwBreakNum = 0;
  localparam bit          DbgTriggerEn  = 1'b0;

  typedef enum int {
    CoreD,
    DbgHost
  } bus_host_e;

  typedef enum int {
    Gpio,
    RpiGpio,
    ArdGpio,
    PmodGpio,
    Pwm,
    Timer,
    RevTags
  } bus_device_e;

  localparam int NrDevices = 7;
  localparam int NrHosts = 2;

  // Interrupts.
  logic timer_irq;
  logic external_irq;

  logic uart0_tx_watermark_irq;
  logic uart0_rx_watermark_irq;
  logic uart0_tx_empty_irq;
  logic uart0_rx_overflow_irq;
  logic uart0_rx_frame_err_irq;
  logic uart0_rx_break_err_irq;
  logic uart0_rx_timeout_irq;
  logic uart0_rx_parity_err_irq;

  logic uart1_tx_watermark_irq;
  logic uart1_rx_watermark_irq;
  logic uart1_tx_empty_irq;
  logic uart1_rx_overflow_irq;
  logic uart1_rx_frame_err_irq;
  logic uart1_rx_break_err_irq;
  logic uart1_rx_timeout_irq;
  logic uart1_rx_parity_err_irq;

  logic uart2_tx_watermark_irq;
  logic uart2_rx_watermark_irq;
  logic uart2_tx_empty_irq;
  logic uart2_rx_overflow_irq;
  logic uart2_rx_frame_err_irq;
  logic uart2_rx_break_err_irq;
  logic uart2_rx_timeout_irq;
  logic uart2_rx_parity_err_irq;

  logic uart3_tx_watermark_irq;
  logic uart3_rx_watermark_irq;
  logic uart3_tx_empty_irq;
  logic uart3_rx_overflow_irq;
  logic uart3_rx_frame_err_irq;
  logic uart3_rx_break_err_irq;
  logic uart3_rx_timeout_irq;
  logic uart3_rx_parity_err_irq;

  logic uart4_tx_watermark_irq;
  logic uart4_rx_watermark_irq;
  logic uart4_tx_empty_irq;
  logic uart4_rx_overflow_irq;
  logic uart4_rx_frame_err_irq;
  logic uart4_rx_break_err_irq;
  logic uart4_rx_timeout_irq;
  logic uart4_rx_parity_err_irq;

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

  logic spi_eth_irq;

  logic [181:0] intr_vector;
  always_comb begin : interrupt_vector
    intr_vector[72 +: 110] = 110'b0;
    intr_vector[71 +: 1] = uart4_rx_parity_err_irq;
    intr_vector[70 +: 1] = uart4_rx_timeout_irq;
    intr_vector[69 +: 1] = uart4_rx_break_err_irq;
    intr_vector[68 +: 1] = uart4_rx_frame_err_irq;
    intr_vector[67 +: 1] = uart4_rx_overflow_irq;
    intr_vector[66 +: 1] = uart4_tx_empty_irq;
    intr_vector[65 +: 1] = uart4_rx_watermark_irq;
    intr_vector[64 +: 1] = uart4_tx_watermark_irq;

    intr_vector[63 +: 1] = uart3_rx_parity_err_irq;
    intr_vector[62 +: 1] = uart3_rx_timeout_irq;
    intr_vector[61 +: 1] = uart3_rx_break_err_irq;
    intr_vector[60 +: 1] = uart3_rx_frame_err_irq;
    intr_vector[59 +: 1] = uart3_rx_overflow_irq;
    intr_vector[58 +: 1] = uart3_tx_empty_irq;
    intr_vector[57 +: 1] = uart3_rx_watermark_irq;
    intr_vector[56 +: 1] = uart3_tx_watermark_irq;

    intr_vector[55 +: 1] = uart2_rx_parity_err_irq;
    intr_vector[54 +: 1] = uart2_rx_timeout_irq;
    intr_vector[53 +: 1] = uart2_rx_break_err_irq;
    intr_vector[52 +: 1] = uart2_rx_frame_err_irq;
    intr_vector[51 +: 1] = uart2_rx_overflow_irq;
    intr_vector[50 +: 1] = uart2_tx_empty_irq;
    intr_vector[49 +: 1] = uart2_rx_watermark_irq;
    intr_vector[48 +: 1] = uart2_tx_watermark_irq;

    intr_vector[47 +: 1] = spi_eth_irq;

    intr_vector[46 +: 1] = i2c1_host_timeout_irq;
    intr_vector[45 +: 1] = i2c1_unexp_stop_irq;
    intr_vector[44 +: 1] = i2c1_acq_full_irq;
    intr_vector[43 +: 1] = i2c1_tx_threshold_irq;
    intr_vector[42 +: 1] = i2c1_tx_stretch_irq;
    intr_vector[41 +: 1] = i2c1_cmd_complete_irq;
    intr_vector[40 +: 1] = i2c1_sda_unstable_irq;
    intr_vector[39 +: 1] = i2c1_stretch_timeout_irq;
    intr_vector[38 +: 1] = i2c1_sda_interference_irq;
    intr_vector[37 +: 1] = i2c1_scl_interference_irq;
    intr_vector[36 +: 1] = i2c1_nak_irq;
    intr_vector[35 +: 1] = i2c1_rx_overflow_irq;
    intr_vector[34 +: 1] = i2c1_acq_threshold_irq;
    intr_vector[33 +: 1] = i2c1_rx_threshold_irq;
    intr_vector[32 +: 1] = i2c1_fmt_threshold_irq;

    intr_vector[31 +: 1] = i2c0_host_timeout_irq;
    intr_vector[30 +: 1] = i2c0_unexp_stop_irq;
    intr_vector[29 +: 1] = i2c0_acq_full_irq;
    intr_vector[28 +: 1] = i2c0_tx_threshold_irq;
    intr_vector[27 +: 1] = i2c0_tx_stretch_irq;
    intr_vector[26 +: 1] = i2c0_cmd_complete_irq;
    intr_vector[25 +: 1] = i2c0_sda_unstable_irq;
    intr_vector[24 +: 1] = i2c0_stretch_timeout_irq;
    intr_vector[23 +: 1] = i2c0_sda_interference_irq;
    intr_vector[22 +: 1] = i2c0_scl_interference_irq;
    intr_vector[21 +: 1] = i2c0_nak_irq;
    intr_vector[20 +: 1] = i2c0_rx_overflow_irq;
    intr_vector[19 +: 1] = i2c0_acq_threshold_irq;
    intr_vector[18 +: 1] = i2c0_rx_threshold_irq;
    intr_vector[17 +: 1] = i2c0_fmt_threshold_irq;

    intr_vector[16 +: 1] = uart1_rx_parity_err_irq;
    intr_vector[15 +: 1] = uart1_rx_timeout_irq;
    intr_vector[14 +: 1] = uart1_rx_break_err_irq;
    intr_vector[13 +: 1] = uart1_rx_frame_err_irq;
    intr_vector[12 +: 1] = uart1_rx_overflow_irq;
    intr_vector[11 +: 1] = uart1_tx_empty_irq;
    intr_vector[10 +: 1] = uart1_rx_watermark_irq;
    intr_vector[9 +: 1]  = uart1_tx_watermark_irq;

    intr_vector[8 +: 1]  = uart0_rx_parity_err_irq;
    intr_vector[7 +: 1]  = uart0_rx_timeout_irq;
    intr_vector[6 +: 1]  = uart0_rx_break_err_irq;
    intr_vector[5 +: 1]  = uart0_rx_frame_err_irq;
    intr_vector[4 +: 1]  = uart0_rx_overflow_irq;
    intr_vector[3 +: 1]  = uart0_tx_empty_irq;
    intr_vector[2 +: 1]  = uart0_rx_watermark_irq;
    intr_vector[1 +: 1]  = uart0_tx_watermark_irq;

    intr_vector[0 +: 1]  = 1'b0; // This is a special case and tied to zero.
  end : interrupt_vector

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
  assign device_req[Gpio]     = device_re[Gpio]     | device_we[Gpio];
  assign device_req[RpiGpio]  = device_re[RpiGpio]  | device_we[RpiGpio];
  assign device_req[ArdGpio]  = device_re[ArdGpio]  | device_we[ArdGpio];
  assign device_req[PmodGpio] = device_re[PmodGpio] | device_we[PmodGpio];
  assign device_req[Pwm]      = device_re[Pwm]      | device_we[Pwm];
  assign device_req[Timer]    = device_re[Timer]    | device_we[Timer];

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

  // Temporal safety signals.
  localparam int unsigned    TsMapAddrWidth = 16;
  logic                      tsmap_cs;
  logic [TsMapAddrWidth-1:0] tsmap_addr;
  logic [BusDataWidth-1:0]   tsmap_rdata;

  // Reset signals
  // Internally generated resets cause IMPERFECTSCH warnings
  /* verilator lint_off IMPERFECTSCH */
  logic rst_core_n;
  logic ndmreset_req;
  /* verilator lint_on IMPERFECTSCH */

  // Hold the core in reset for a period after debug monitor accesses,
  // in anticipation of further accesses whilst downloading completes.
  //
  // Release the code after 2^21 cycles, which will be ca. 20-100ms for
  // typical system clock frequencies.
  logic [20:0] dbg_release_cnt;
  wire dbg_release_core = &dbg_release_cnt;
  always_ff @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      dbg_release_cnt  <= {21{1'b1}};
    end else if (host_req[DbgHost] | ~dbg_release_core) begin
      dbg_release_cnt  <= host_req[DbgHost] ? '0 : (dbg_release_cnt + 1);
    end
  end

  // Tie-off unused error signals.
  assign device_err[Gpio]     = 1'b0;
  assign device_err[RpiGpio]  = 1'b0;
  assign device_err[ArdGpio]  = 1'b0;
  assign device_err[PmodGpio] = 1'b0;
  assign device_err[Pwm]      = 1'b0;

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
  tlul_pkg::tl_h2d_t tl_rpi_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_rpi_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_ard_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_ard_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_pmod_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_pmod_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_uart0_h2d;
  tlul_pkg::tl_d2h_t tl_uart0_d2h;
  tlul_pkg::tl_h2d_t tl_uart1_h2d;
  tlul_pkg::tl_d2h_t tl_uart1_d2h;
  tlul_pkg::tl_h2d_t tl_uart2_h2d;
  tlul_pkg::tl_d2h_t tl_uart2_d2h;
  tlul_pkg::tl_h2d_t tl_uart3_h2d;
  tlul_pkg::tl_d2h_t tl_uart3_d2h;
  tlul_pkg::tl_h2d_t tl_uart4_h2d;
  tlul_pkg::tl_d2h_t tl_uart4_d2h;
  tlul_pkg::tl_h2d_t tl_timer_h2d;
  tlul_pkg::tl_d2h_t tl_timer_d2h;
  tlul_pkg::tl_h2d_t tl_rgbled_ctrl_h2d;
  tlul_pkg::tl_d2h_t tl_rgbled_ctrl_d2h;
  tlul_pkg::tl_h2d_t tl_pwm_h2d;
  tlul_pkg::tl_d2h_t tl_pwm_d2h;
  tlul_pkg::tl_h2d_t tl_i2c0_h2d;
  tlul_pkg::tl_d2h_t tl_i2c0_d2h;
  tlul_pkg::tl_h2d_t tl_i2c1_h2d;
  tlul_pkg::tl_d2h_t tl_i2c1_d2h;
  tlul_pkg::tl_h2d_t tl_rv_plic_h2d;
  tlul_pkg::tl_d2h_t tl_rv_plic_d2h;
  tlul_pkg::tl_h2d_t tl_spi_flash_h2d;
  tlul_pkg::tl_d2h_t tl_spi_flash_d2h;
  tlul_pkg::tl_h2d_t tl_spi_lcd_h2d;
  tlul_pkg::tl_d2h_t tl_spi_lcd_d2h;
  tlul_pkg::tl_h2d_t tl_spi_eth_h2d;
  tlul_pkg::tl_d2h_t tl_spi_eth_d2h;
  tlul_pkg::tl_h2d_t tl_spi_rp0_h2d;
  tlul_pkg::tl_d2h_t tl_spi_rp0_d2h;
  tlul_pkg::tl_h2d_t tl_spi_rp1_h2d;
  tlul_pkg::tl_d2h_t tl_spi_rp1_d2h;
  tlul_pkg::tl_h2d_t tl_spi_ard_h2d;
  tlul_pkg::tl_d2h_t tl_spi_ard_d2h;
  tlul_pkg::tl_h2d_t tl_spi_mkr_h2d;
  tlul_pkg::tl_d2h_t tl_spi_mkr_d2h;
  tlul_pkg::tl_h2d_t tl_usbdev_h2d;
  tlul_pkg::tl_d2h_t tl_usbdev_d2h;
  tlul_pkg::tl_h2d_t tl_rev_h2d;
  tlul_pkg::tl_d2h_t tl_rev_d2h;

  xbar_main xbar (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),
    .clk_usb_i        (clk_usb_i),
    .rst_usb_ni       (rst_usb_ni),

    // Host interfaces.
    .tl_ibex_lsu_i    (tl_ibex_lsu_h2d_q),
    .tl_ibex_lsu_o    (tl_ibex_lsu_d2h_q),
    .tl_dbg_host_i    (tl_dbg_host_h2d_q),
    .tl_dbg_host_o    (tl_dbg_host_d2h_q),

    // Device interfaces.
    .tl_sram_o        (tl_sram_h2d_d),
    .tl_sram_i        (tl_sram_d2h_d),
    .tl_rev_tag_o     (tl_rev_h2d),
    .tl_rev_tag_i     (tl_rev_d2h),
    .tl_gpio_o        (tl_gpio_h2d),
    .tl_gpio_i        (tl_gpio_d2h),
    .tl_pwm_o         (tl_pwm_h2d),
    .tl_pwm_i         (tl_pwm_d2h),
    .tl_rpi_gpio_o    (tl_rpi_gpio_h2d),
    .tl_rpi_gpio_i    (tl_rpi_gpio_d2h),
    .tl_ard_gpio_o    (tl_ard_gpio_h2d),
    .tl_ard_gpio_i    (tl_ard_gpio_d2h),
    .tl_pmod_gpio_o   (tl_pmod_gpio_h2d),
    .tl_pmod_gpio_i   (tl_pmod_gpio_d2h),
    .tl_rgbled_ctrl_o (tl_rgbled_ctrl_h2d),
    .tl_rgbled_ctrl_i (tl_rgbled_ctrl_d2h),
    .tl_timer_o       (tl_timer_h2d),
    .tl_timer_i       (tl_timer_d2h),
    .tl_uart0_o       (tl_uart0_h2d),
    .tl_uart0_i       (tl_uart0_d2h),
    .tl_uart1_o       (tl_uart1_h2d),
    .tl_uart1_i       (tl_uart1_d2h),
    .tl_uart2_o       (tl_uart2_h2d),
    .tl_uart2_i       (tl_uart2_d2h),
    .tl_uart3_o       (tl_uart3_h2d),
    .tl_uart3_i       (tl_uart3_d2h),
    .tl_uart4_o       (tl_uart4_h2d),
    .tl_uart4_i       (tl_uart4_d2h),
    .tl_i2c0_o        (tl_i2c0_h2d),
    .tl_i2c0_i        (tl_i2c0_d2h),
    .tl_i2c1_o        (tl_i2c1_h2d),
    .tl_i2c1_i        (tl_i2c1_d2h),
    .tl_spi_flash_o   (tl_spi_flash_h2d),
    .tl_spi_flash_i   (tl_spi_flash_d2h),
    .tl_spi_lcd_o     (tl_spi_lcd_h2d),
    .tl_spi_lcd_i     (tl_spi_lcd_d2h),
    .tl_spi_eth_o     (tl_spi_eth_h2d),
    .tl_spi_eth_i     (tl_spi_eth_d2h),
    .tl_spi_rp0_o     (tl_spi_rp0_h2d),
    .tl_spi_rp0_i     (tl_spi_rp0_d2h),
    .tl_spi_rp1_o     (tl_spi_rp1_h2d),
    .tl_spi_rp1_i     (tl_spi_rp1_d2h),
    .tl_spi_ard_o     (tl_spi_ard_h2d),
    .tl_spi_ard_i     (tl_spi_ard_d2h),
    .tl_spi_mkr_o     (tl_spi_mkr_h2d),
    .tl_spi_mkr_i     (tl_spi_mkr_d2h),
    .tl_usbdev_o      (tl_usbdev_h2d),
    .tl_usbdev_i      (tl_usbdev_d2h),
    .tl_rv_plic_o     (tl_rv_plic_h2d),
    .tl_rv_plic_i     (tl_rv_plic_d2h),

    .scanmode_i       (prim_mubi_pkg::MuBi4False)
  );

  // TL-UL host adapter(s).

  tlul_adapter_host ibex_ins_host_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    .req_i        (core_instr_req_filtered),
    .gnt_o        (core_instr_gnt),
    .addr_i       (core_instr_addr),
    .we_i         ('0),
    .wdata_i      ('0),
    .wdata_cap_i  ('0),
    .wdata_intg_i ('0),
    .be_i         ('0),
    .instr_type_i (prim_mubi_pkg::MuBi4True),

    .valid_o      (core_instr_rvalid),
    .rdata_o      (core_instr_rdata),
    .rdata_cap_o  (), // Instructions should not have capability tag set.
    .rdata_intg_o (),
    .err_o        (core_instr_err),
    .intg_err_o   (),

    .tl_o         (tl_ibex_ins_h2d),
    .tl_i         (tl_ibex_ins_d2h)
  );

  tlul_adapter_host ibex_lsu_host_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    .req_i        (host_req[CoreD]),
    .gnt_o        (host_gnt[CoreD]),
    .addr_i       (host_addr[CoreD]),
    .we_i         (host_we[CoreD]),
    .wdata_i      (host_wdata[CoreD]),
    .wdata_cap_i  (host_wcap[CoreD]),
    .wdata_intg_i ('0),
    .be_i         (host_be[CoreD]),
    .instr_type_i (prim_mubi_pkg::MuBi4False),

    .valid_o      (host_rvalid[CoreD]),
    .rdata_o      (host_rdata[CoreD]),
    .rdata_cap_o  (host_rcap[CoreD]),
    .rdata_intg_o (),
    .err_o        (host_err[CoreD]),
    .intg_err_o   (),

    .tl_o         (tl_ibex_lsu_h2d_d),
    .tl_i         (tl_ibex_lsu_d2h_d)
  );

  tlul_adapter_host dbg_host_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    .req_i        (host_req[DbgHost]),
    .gnt_o        (host_gnt[DbgHost]),
    .addr_i       (host_addr[DbgHost]),
    .we_i         (host_we[DbgHost]),
    .wdata_i      (host_wdata[DbgHost]),
    .wdata_cap_i  (host_wcap[DbgHost]),
    .wdata_intg_i ('0),
    .be_i         (host_be[DbgHost]),
    .instr_type_i (prim_mubi_pkg::MuBi4False),

    .valid_o      (host_rvalid[DbgHost]),
    .rdata_o      (host_rdata[DbgHost]),
    .rdata_cap_o  (host_rcap[DbgHost]),
    .rdata_intg_o (),
    .err_o        (host_err[DbgHost]),
    .intg_err_o   (),

    .tl_o         (tl_dbg_host_h2d_d),
    .tl_i         (tl_dbg_host_d2h_d)
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_ibex_lsu_fifo (
    .clk_i       (clk_sys_i),
    .rst_ni      (rst_sys_ni),

    .tl_h_i      (tl_ibex_lsu_h2d_d),
    .tl_h_o      (tl_ibex_lsu_d2h_d),
    .tl_d_o      (tl_ibex_lsu_h2d_q),
    .tl_d_i      (tl_ibex_lsu_d2h_q),

    .spare_req_i (1'b0),
    .spare_req_o (    ),
    .spare_rsp_i (1'b0),
    .spare_rsp_o (    )
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_dbg_host_fifo (
    .clk_i       (clk_sys_i),
    .rst_ni      (rst_sys_ni),

    .tl_h_i      (tl_dbg_host_h2d_d),
    .tl_h_o      (tl_dbg_host_d2h_d),
    .tl_d_o      (tl_dbg_host_h2d_q),
    .tl_d_i      (tl_dbg_host_d2h_q),

    .spare_req_i (1'b0),
    .spare_req_o (    ),
    .spare_rsp_i (1'b0),
    .spare_rsp_o (    )
  );

  // This latch is necessary to avoid circular logic. This shows up as an `UNOPTFLAT` warning in Verilator.
  tlul_fifo_sync #(
    .ReqPass  ( 0 ),
    .RspPass  ( 0 ),
    .ReqDepth ( 2 ),
    .RspDepth ( 2 )
  ) tl_sram_fifo (
    .clk_i       (clk_sys_i),
    .rst_ni      (rst_sys_ni),

    .tl_h_i      (tl_sram_h2d_d),
    .tl_h_o      (tl_sram_d2h_d),
    .tl_d_o      (tl_sram_h2d_q),
    .tl_d_i      (tl_sram_d2h_q),

    .spare_req_i (1'b0),
    .spare_req_o (    ),
    .spare_rsp_i (1'b0),
    .spare_rsp_o (    )
  );

  sram #(
    .AddrWidth       ( SRAMAddrWidth   ),
    .DataWidth       ( BusDataWidth    ),
    .DataBitsPerMask ( DataBitsPerMask ),
    .InitFile        ( SRAMInitFile    )
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
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_gpio_h2d),
    .tl_o         (tl_gpio_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[Gpio]),
    .we_o         (device_we[Gpio]),
    .addr_o       (device_addr[Gpio][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[Gpio]),
    .be_o         (device_be[Gpio]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[Gpio]),
    .error_i      (device_err[Gpio])
  );

  // Tie off upper bits of address.
  assign device_addr[Gpio][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) rpi_gpio_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_rpi_gpio_h2d),
    .tl_o         (tl_rpi_gpio_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[RpiGpio]),
    .we_o         (device_we[RpiGpio]),
    .addr_o       (device_addr[RpiGpio][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[RpiGpio]),
    .be_o         (device_be[RpiGpio]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[RpiGpio]),
    .error_i      (device_err[RpiGpio])
  );

  // Tie off upper bits of address.
  assign device_addr[RpiGpio][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) ard_gpio_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_ard_gpio_h2d),
    .tl_o         (tl_ard_gpio_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[ArdGpio]),
    .we_o         (device_we[ArdGpio]),
    .addr_o       (device_addr[ArdGpio][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[ArdGpio]),
    .be_o         (device_be[ArdGpio]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[ArdGpio]),
    .error_i      (device_err[ArdGpio])
  );

  // Tie off upper bits of address.
  assign device_addr[ArdGpio][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) pmod_gpio_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_pmod_gpio_h2d),
    .tl_o         (tl_pmod_gpio_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[PmodGpio]),
    .we_o         (device_we[PmodGpio]),
    .addr_o       (device_addr[PmodGpio][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[PmodGpio]),
    .be_o         (device_be[PmodGpio]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[PmodGpio]),
    .error_i      (device_err[PmodGpio])
  );

  // Tie off upper bits of address.
  assign device_addr[PmodGpio][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) pwm_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_pwm_h2d),
    .tl_o         (tl_pwm_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[Pwm]),
    .we_o         (device_we[Pwm]),
    .addr_o       (device_addr[Pwm][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[Pwm]),
    .be_o         (device_be[Pwm]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[Pwm]),
    .error_i      (device_err[Pwm])
  );

  // Tie off upper bits of address.
  assign device_addr[Pwm][BusAddrWidth-1:RegAddrWidth] = '0;

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .RegAw            ( TRegAddrWidth ),
    .AccessLatency    ( 1 )
  ) timer_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_timer_h2d),
    .tl_o         (tl_timer_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[Timer]),
    .we_o         (device_we[Timer]),
    .addr_o       (device_addr[Timer][TRegAddrWidth-1:0]),
    .wdata_o      (device_wdata[Timer]),
    .be_o         (device_be[Timer]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[Timer]),
    .error_i      (device_err[Timer])
  );

  // Tie off upper bits of address.
  assign device_addr[Timer][BusAddrWidth-1:TRegAddrWidth] = '0;

  // Revocation tag memory.
  logic [BusDataWidth-1:0] revocation_tags_bit_enable;

  always_ff @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      device_rvalid[RevTags] <= 1'b0;
    end else begin
      device_rvalid[RevTags] <= device_req[RevTags] & ~device_we[RevTags];
    end
  end

  // Size of revocation tag memory is 4 KiB, one bit for each 64 in SRAM
  localparam int unsigned RevTagDepth = 4 * 1024 * 8 / BusDataWidth;
  localparam int unsigned RevTagAddrWidth = $clog2(RevTagDepth);

  tlul_adapter_sram #(
    .SramAw           ( RevTagAddrWidth ),
    .EnableRspIntgGen ( 1               )
  ) revocation_sram_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_rev_h2d),
    .tl_o         (tl_rev_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),

    // SRAM interface.
    .req_o        (device_req[RevTags]),
    .req_type_o   (),
    .gnt_i        (device_req[RevTags]),
    .we_o         (device_we[RevTags]),
    .addr_o       (device_addr[RevTags][RevTagAddrWidth-1:0]),
    .wdata_o      (device_wdata[RevTags]),
    .wdata_cap_o  (),
    .wmask_o      (revocation_tags_bit_enable),
    .intg_error_o (),
    .rdata_i      (device_rdata[RevTags]),
    .rdata_cap_i  (1'b0),
    .rvalid_i     (device_rvalid[RevTags]),
    .rerror_i     (2'b00)
  );

  // Tie off upper bits of address.
  assign device_addr[RevTags][BusAddrWidth-1:RevTagAddrWidth] = '0;

  prim_ram_2p #(
    .Depth           ( RevTagDepth     ),
    .Width           ( BusDataWidth    ),
    .DataBitsPerMask ( DataBitsPerMask )
  ) u_revocation_ram (
    .clk_a_i   (clk_sys_i),
    .clk_b_i   (clk_sys_i),
    .cfg_i     ('0),
    .a_req_i   (device_req[RevTags]),
    .a_write_i (device_we[RevTags]),
    .a_addr_i  (device_addr[RevTags][RevTagAddrWidth-1:0]),
    .a_wdata_i (device_wdata[RevTags]),
    .a_wmask_i (revocation_tags_bit_enable),
    .a_rdata_o (device_rdata[RevTags]),
    .b_req_i   (tsmap_cs),
    .b_write_i (1'b0),
    .b_wmask_i ('0),
    .b_addr_i  (tsmap_addr[RevTagAddrWidth-1:0]),
    .b_wdata_i ('0),
    .b_rdata_o (tsmap_rdata)
  );

  ///////////////////////////////////////////////
  // Core and hardware IP block instantiation. //
  ///////////////////////////////////////////////

  logic cheri_en;

  assign cheri_en   = cheri_en_i;
  assign cheri_en_o = cheri_en;
  assign rst_core_n = rst_sys_ni & ~ndmreset_req & dbg_release_core;

  logic [CheriErrWidth-1:0] cheri_err;

  for (genvar i = 0; i < CheriErrWidth; ++i) begin : gen_pwm_fade
    pwm_fade u_pwm_fade (
      .clk_i       (clk_sys_i      ),
      .rst_ni      (rst_core_n     ),
      .impulse_i   (cheri_err[i]   ),
      .modulated_o (cheri_err_o[i] )
    );
  end

  ibexc_top_tracing #(
    .DmHaltAddr      ( DebugStart + dm::HaltAddress[31:0]      ),
    .DmExceptionAddr ( DebugStart + dm::ExceptionAddress[31:0] ),
    .DbgTriggerEn    ( DbgTriggerEn                            ),
    .DbgHwBreakNum   ( DbgHwBreakNum                           ),
    .MHPMCounterNum  ( 13                                      ),
    // For now revocation tags apply to all of SRAM.
    .HeapBase        ( tl_main_pkg::ADDR_SPACE_SRAM            ),
    .TSMapBase       ( tl_main_pkg::ADDR_SPACE_REV_TAG         ),
    .TSMapSize       ( RevTagDepth                             ),
    .RV32B           ( ibex_pkg::RV32BFull                     )
  ) u_top_tracing (
    .clk_i                  (clk_sys_i),
    .rst_ni                 (rst_core_n),

    .test_en_i              (1'b0),
    .scan_rst_ni            (1'b1),
    .ram_cfg_i              (10'b0),

    .cheri_pmode_i          (cheri_en),
    .cheri_tsafe_en_i       (cheri_en),
    .cheri_err_o            (cheri_err),

    .hart_id_i              (32'b0),
    // First instruction executed is at 0x0010_0000 + 0x80.
    .boot_addr_i            (32'h0010_0000),

    .instr_req_o            (core_instr_req),
    .instr_gnt_i            (core_instr_gnt),
    .instr_rvalid_i         (core_instr_rvalid),
    .instr_addr_o           (core_instr_addr),
    .instr_rdata_i          (core_instr_rdata),
    .instr_rdata_intg_i     ('0),
    .instr_err_i            (core_instr_err),

    .data_req_o             (host_req[CoreD]),
    .data_is_cap_o          (),
    .data_gnt_i             (host_gnt[CoreD]),
    .data_rvalid_i          (host_rvalid[CoreD]),
    .data_we_o              (host_we[CoreD]),
    .data_be_o              (host_be[CoreD]),
    .data_addr_o            (host_addr[CoreD]),
    .data_wdata_o           (cheri_wdata),
    .data_wdata_intg_o      (),
    .data_rdata_i           (cheri_rdata),
    .data_rdata_intg_i      ('0),
    .data_err_i             (host_err[CoreD]),

    .tsmap_cs_o             (tsmap_cs),
    .tsmap_addr_o           (tsmap_addr),
    .tsmap_rdata_i          (tsmap_rdata),

    // TODO fill this in to control hardware revocation engine.
    .mmreg_corein_i         (128'b0),
    .mmreg_coreout_o        (),
    .cheri_fatal_err_o      (),

    .irq_software_i         (1'b0),
    .irq_timer_i            (timer_irq),
    .irq_external_i         (external_irq),
    .irq_fast_i             (15'b0),
    .irq_nm_i               (1'b0),

    .scramble_key_valid_i   ('0),
    .scramble_key_i         ('0),
    .scramble_nonce_i       ('0),
    .scramble_req_o         (  ),

    .debug_req_i            (),
    .crash_dump_o           (),
    .double_fault_seen_o    (),

    .fetch_enable_i         ('1),
    .alert_minor_o          (  ),
    .alert_major_internal_o (  ),
    .alert_major_bus_o      (  ),
    .core_sleep_o           (  )
  );

  gpio #(
    .GpiWidth ( GpiWidth ),
    .GpoWidth ( GpoWidth )
  ) u_gpio (
    .clk_i           (clk_sys_i),
    .rst_ni          (rst_sys_ni),

    // Bus interface.
    .device_req_i    (device_req[Gpio]),
    .device_addr_i   (device_addr[Gpio]),
    .device_we_i     (device_we[Gpio]),
    .device_be_i     (device_be[Gpio]),
    .device_wdata_i  (device_wdata[Gpio]),
    .device_rvalid_o (device_rvalid[Gpio]),
    .device_rdata_o  (device_rdata[Gpio]),

    .gp_i,
    .gp_o
  );

  // GPIO for the Raspberry Pi HAT.
  gpio #(
    .GpiWidth ( RpiGpiWidth ),
    .GpoWidth ( WordWidth   )
  ) u_rpi_gpio (
    .clk_i           (clk_sys_i),
    .rst_ni          (rst_sys_ni),

    // Bus interface.
    .device_req_i    (device_req[RpiGpio]),
    .device_addr_i   (device_addr[RpiGpio]),
    .device_we_i     (device_we[RpiGpio]),
    .device_be_i     (device_be[RpiGpio]),
    .device_wdata_i  (device_wdata[RpiGpio]),
    .device_rvalid_o (device_rvalid[RpiGpio]),
    .device_rdata_o  (device_rdata[RpiGpio]),

    .gp_i            (rp_gp_i),
    .gp_o            (rp_gp_o)
  );

  // GPIO for the Arduino Shield.
  gpio #(
    .GpiWidth ( ArdGpiWidth ),
    .GpoWidth ( WordWidth   )
  ) u_ard_gpio (
    .clk_i           (clk_sys_i),
    .rst_ni          (rst_sys_ni),

    // Bus interface.
    .device_req_i    (device_req[ArdGpio]),
    .device_addr_i   (device_addr[ArdGpio]),
    .device_we_i     (device_we[ArdGpio]),
    .device_be_i     (device_be[ArdGpio]),
    .device_wdata_i  (device_wdata[ArdGpio]),
    .device_rvalid_o (device_rvalid[ArdGpio]),
    .device_rdata_o  (device_rdata[ArdGpio]),

    .gp_i            (ard_gp_i),
    .gp_o            (ard_gp_o)
  );

  // GPIO for PMOD connectors.
  gpio #(
    .GpiWidth ( PmodGpiWidth ),
    .GpoWidth ( WordWidth    )
  ) u_pmod_gpio (
    .clk_i           (clk_sys_i),
    .rst_ni          (rst_sys_ni),

    // Bus interface.
    .device_req_i    (device_req[PmodGpio]),
    .device_addr_i   (device_addr[PmodGpio]),
    .device_we_i     (device_we[PmodGpio]),
    .device_be_i     (device_be[PmodGpio]),
    .device_wdata_i  (device_wdata[PmodGpio]),
    .device_rvalid_o (device_rvalid[PmodGpio]),
    .device_rdata_o  (device_rdata[PmodGpio]),

    .gp_i            (pmod_gp_i),
    .gp_o            (pmod_gp_o)
  );

  i2c u_i2c0 (
      .clk_i                   (clk_sys_i),
      .rst_ni                  (rst_sys_ni),
      .ram_cfg_i               (10'b0),

      // Bus interface.
      .tl_i                    (tl_i2c0_h2d),
      .tl_o                    (tl_i2c0_d2h),

      // Generic IO.
      .cio_scl_i               (i2c0_scl_i),
      .cio_scl_o               (i2c0_scl_o),
      .cio_scl_en_o            (i2c0_scl_en_o),
      .cio_sda_i               (i2c0_sda_i),
      .cio_sda_o               (i2c0_sda_o),
      .cio_sda_en_o            (i2c0_sda_en_o),

      // Interrupts.
      .intr_fmt_threshold_o    (i2c0_fmt_threshold_irq),
      .intr_rx_threshold_o     (i2c0_rx_threshold_irq),
      .intr_acq_threshold_o    (i2c0_acq_threshold_irq),
      .intr_rx_overflow_o      (i2c0_rx_overflow_irq),
      .intr_nak_o              (i2c0_nak_irq),
      .intr_scl_interference_o (i2c0_scl_interference_irq),
      .intr_sda_interference_o (i2c0_sda_interference_irq),
      .intr_stretch_timeout_o  (i2c0_stretch_timeout_irq),
      .intr_sda_unstable_o     (i2c0_sda_unstable_irq),
      .intr_cmd_complete_o     (i2c0_cmd_complete_irq),
      .intr_tx_stretch_o       (i2c0_tx_stretch_irq),
      .intr_tx_threshold_o     (i2c0_tx_threshold_irq),
      .intr_acq_full_o         (i2c0_acq_full_irq),
      .intr_unexp_stop_o       (i2c0_unexp_stop_irq),
      .intr_host_timeout_o     (i2c0_host_timeout_irq)
  );

  i2c u_i2c1 (
      .clk_i                   (clk_sys_i),
      .rst_ni                  (rst_sys_ni),
      .ram_cfg_i               (10'b0),

      // Bus interface.
      .tl_i                    (tl_i2c1_h2d),
      .tl_o                    (tl_i2c1_d2h),

      // Generic IO.
      .cio_scl_i               (i2c1_scl_i),
      .cio_scl_o               (i2c1_scl_o),
      .cio_scl_en_o            (i2c1_scl_en_o),
      .cio_sda_i               (i2c1_sda_i),
      .cio_sda_o               (i2c1_sda_o),
      .cio_sda_en_o            (i2c1_sda_en_o),

      // Interrupts.
      .intr_fmt_threshold_o    (i2c1_fmt_threshold_irq),
      .intr_rx_threshold_o     (i2c1_rx_threshold_irq),
      .intr_acq_threshold_o    (i2c1_acq_threshold_irq),
      .intr_rx_overflow_o      (i2c1_rx_overflow_irq),
      .intr_nak_o              (i2c1_nak_irq),
      .intr_scl_interference_o (i2c1_scl_interference_irq),
      .intr_sda_interference_o (i2c1_sda_interference_irq),
      .intr_stretch_timeout_o  (i2c1_stretch_timeout_irq),
      .intr_sda_unstable_o     (i2c1_sda_unstable_irq),
      .intr_cmd_complete_o     (i2c1_cmd_complete_irq),
      .intr_tx_stretch_o       (i2c1_tx_stretch_irq),
      .intr_tx_threshold_o     (i2c1_tx_threshold_irq),
      .intr_acq_full_o         (i2c1_acq_full_irq),
      .intr_unexp_stop_o       (i2c1_unexp_stop_irq),
      .intr_host_timeout_o     (i2c1_host_timeout_irq)
  );

  // Pulse width modulator.
  pwm_wrapper #(
    .PwmWidth   ( PwmWidth   ),
    .PwmCtrSize ( PwmCtrSize )
  ) u_pwm (
    .clk_i           (clk_sys_i),
    .rst_ni          (rst_sys_ni),

    .device_req_i    (device_req[Pwm]),
    .device_addr_i   (device_addr[Pwm]),
    .device_we_i     (device_we[Pwm]),
    .device_be_i     (device_be[Pwm]),
    .device_wdata_i  (device_wdata[Pwm]),
    .device_rvalid_o (device_rvalid[Pwm]),
    .device_rdata_o  (device_rdata[Pwm]),

    .pwm_o
  );

  uart u_uart0 (
      .clk_i                (clk_sys_i  ),
      .rst_ni               (rst_sys_ni ),

      .cio_rx_i             (uart0_rx_i ),
      .cio_tx_o             (uart0_tx_o ),
      .cio_tx_en_o          (           ),

      // Inter-module signals.
      .tl_i                 (tl_uart0_h2d),
      .tl_o                 (tl_uart0_d2h),

      // Interrupt.
      .intr_tx_watermark_o  (uart0_tx_watermark_irq),
      .intr_rx_watermark_o  (uart0_rx_watermark_irq),
      .intr_tx_empty_o      (uart0_tx_empty_irq),
      .intr_rx_overflow_o   (uart0_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart0_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart0_rx_break_err_irq),
      .intr_rx_timeout_o    (uart0_rx_timeout_irq),
      .intr_rx_parity_err_o (uart0_rx_parity_err_irq)
  );

  uart u_uart1 (
      .clk_i                (clk_sys_i  ),
      .rst_ni               (rst_sys_ni ),

      .cio_rx_i             (uart1_rx_i ),
      .cio_tx_o             (uart1_tx_o ),
      .cio_tx_en_o          (           ),

      // Inter-module signals
      .tl_i                 (tl_uart1_h2d),
      .tl_o                 (tl_uart1_d2h),

      // Interrupt
      .intr_tx_watermark_o  (uart1_tx_watermark_irq),
      .intr_rx_watermark_o  (uart1_rx_watermark_irq),
      .intr_tx_empty_o      (uart1_tx_empty_irq),
      .intr_rx_overflow_o   (uart1_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart1_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart1_rx_break_err_irq),
      .intr_rx_timeout_o    (uart1_rx_timeout_irq),
      .intr_rx_parity_err_o (uart1_rx_parity_err_irq)
  );

  uart u_uart2 (
      .clk_i                (clk_sys_i  ),
      .rst_ni               (rst_sys_ni ),

      .cio_rx_i             (uart2_rx_i ),
      .cio_tx_o             (uart2_tx_o ),
      .cio_tx_en_o          (           ),

      // Inter-module signals
      .tl_i                 (tl_uart2_h2d),
      .tl_o                 (tl_uart2_d2h),

      // Interrupt
      .intr_tx_watermark_o  (uart2_tx_watermark_irq),
      .intr_rx_watermark_o  (uart2_rx_watermark_irq),
      .intr_tx_empty_o      (uart2_tx_empty_irq),
      .intr_rx_overflow_o   (uart2_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart2_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart2_rx_break_err_irq),
      .intr_rx_timeout_o    (uart2_rx_timeout_irq),
      .intr_rx_parity_err_o (uart2_rx_parity_err_irq)
  );

  uart u_uart3 (
      .clk_i                (clk_sys_i  ),
      .rst_ni               (rst_sys_ni ),

      .cio_rx_i             (uart3_rx_i ),
      .cio_tx_o             (uart3_tx_o ),
      .cio_tx_en_o          (           ),

      // Inter-module signals
      .tl_i                 (tl_uart3_h2d),
      .tl_o                 (tl_uart3_d2h),

      // Interrupt
      .intr_tx_watermark_o  (uart3_tx_watermark_irq),
      .intr_rx_watermark_o  (uart3_rx_watermark_irq),
      .intr_tx_empty_o      (uart3_tx_empty_irq),
      .intr_rx_overflow_o   (uart3_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart3_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart3_rx_break_err_irq),
      .intr_rx_timeout_o    (uart3_rx_timeout_irq),
      .intr_rx_parity_err_o (uart3_rx_parity_err_irq)
  );

  uart u_uart4 (
      .clk_i                (clk_sys_i  ),
      .rst_ni               (rst_sys_ni ),

      .cio_rx_i             (uart4_rx_i ),
      .cio_tx_o             (uart4_tx_o ),
      .cio_tx_en_o          (           ),

      // Inter-module signals
      .tl_i                 (tl_uart4_h2d),
      .tl_o                 (tl_uart4_d2h),

      // Interrupt
      .intr_tx_watermark_o  (uart4_tx_watermark_irq),
      .intr_rx_watermark_o  (uart4_rx_watermark_irq),
      .intr_tx_empty_o      (uart4_tx_empty_irq),
      .intr_rx_overflow_o   (uart4_rx_overflow_irq),
      .intr_rx_frame_err_o  (uart4_rx_frame_err_irq),
      .intr_rx_break_err_o  (uart4_rx_break_err_irq),
      .intr_rx_timeout_o    (uart4_rx_timeout_irq),
      .intr_rx_parity_err_o (uart4_rx_parity_err_irq)
  );

  // USB device.
  usbdev #(
    .Stub ( 1'b0 )
  ) u_usbdev (
    .clk_i                        (clk_usb_i),
    .rst_ni                       (rst_usb_ni),

    // AON Wakeup functionality is not being used
    .clk_aon_i                    (clk_usb_i),
    .rst_aon_ni                   (rst_usb_ni),

    .tl_i                         (tl_usbdev_h2d),
    .tl_o                         (tl_usbdev_d2h),

    // Data inputs
    .cio_usb_dp_i                 (usb_dp_i),
    .cio_usb_dn_i                 (usb_dn_i),
    .usb_rx_d_i                   (usb_rx_d_i),

    // Data outputs
    .cio_usb_dp_o                 (usb_dp_o),
    .cio_usb_dp_en_o              (usb_dp_en_o),
    .cio_usb_dn_o                 (usb_dn_o),
    .cio_usb_dn_en_o              (usb_dn_en_o),
    .usb_tx_se0_o                 (),
    .usb_tx_d_o                   (),
 
    // Non-data I/O
    .cio_sense_i                  (usb_sense_i),
    .usb_dp_pullup_o              (usb_dp_pullup_o),
    .usb_dn_pullup_o              (usb_dn_pullup_o),
    .usb_rx_enable_o              (usb_rx_enable_o),
    .usb_tx_use_d_se0_o           (),

    // Unused AON/Wakeup functionality
    .usb_aon_suspend_req_o        (),
    .usb_aon_wake_ack_o           (),

    .usb_aon_bus_reset_i          (1'b0),
    .usb_aon_sense_lost_i         (1'b0),
    .usb_aon_bus_not_idle_i       (1'b0),
    .usb_aon_wake_detect_active_i (1'b0),

    .usb_ref_val_o                (),
    .usb_ref_pulse_o              (),

    .ram_cfg_i                    (10'b0),

    // Interrupts not required
    .intr_pkt_received_o          (),
    .intr_pkt_sent_o              (),
    .intr_powered_o               (),
    .intr_disconnected_o          (),
    .intr_host_lost_o             (),
    .intr_link_reset_o            (),
    .intr_link_suspend_o          (),
    .intr_link_resume_o           (),
    .intr_av_out_empty_o          (),
    .intr_rx_full_o               (),
    .intr_av_overflow_o           (),
    .intr_link_in_err_o           (),
    .intr_link_out_err_o          (),
    .intr_rx_crc_err_o            (),
    .intr_rx_pid_err_o            (),
    .intr_rx_bitstuff_err_o       (),
    .intr_frame_o                 (),
    .intr_av_setup_empty_o        ()
  );

  // SPI host for talking to Flash memory.
  spi u_spi_flash (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    // TileLink interface.
    .tl_i                (tl_spi_flash_h2d),
    .tl_o                (tl_spi_flash_d2h),

    // Interrupts currently disconnected.
    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    // SPI signals.
    .spi_copi_o          (spi_flash_tx_o),
    .spi_cipo_i          (spi_flash_rx_i),
    .spi_clk_o           (spi_flash_sck_o)
  );

  // SPI host for writing to the LCD screen.
  spi u_spi_lcd (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    // TileLink interface.
    .tl_i                (tl_spi_lcd_h2d),
    .tl_o                (tl_spi_lcd_d2h),

    // Interrupts currently disconnected.
    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    // SPI signals.
    .spi_copi_o          (spi_lcd_tx_o),
    .spi_cipo_i          (spi_lcd_rx_i),
    .spi_clk_o           (spi_lcd_sck_o)
  );

  // SPI host for talking to ethernet chip.
  spi u_spi_eth (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    // TileLink interface.
    .tl_i                (tl_spi_eth_h2d),
    .tl_o                (tl_spi_eth_d2h),

    // Interrupts currently disconnected.
    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    // SPI signals.
    .spi_copi_o          (spi_eth_tx_o),
    .spi_cipo_i          (spi_eth_rx_i),
    .spi_clk_o           (spi_eth_sck_o)
  );

  // Sample the ethernet interrupt pin.
  always_ff @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      spi_eth_irq <= 1'b0;
    end else begin
      spi_eth_irq <= !spi_eth_irq_ni;
    end
  end

  // Host for SPI0 on the Raspberry Pi HAT.
  spi u_spi_rp0 (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    .tl_i                (tl_spi_rp0_h2d),
    .tl_o                (tl_spi_rp0_d2h),

    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    .spi_copi_o          (spi_rp0_tx_o),
    .spi_cipo_i          (spi_rp0_rx_i),
    .spi_clk_o           (spi_rp0_sck_o)
  );

  // Host for SPI1 on the Raspberry Pi HAT.
  spi u_spi_rp1 (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    .tl_i                (tl_spi_rp1_h2d),
    .tl_o                (tl_spi_rp1_d2h),

    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    .spi_copi_o          (spi_rp1_tx_o),
    .spi_cipo_i          (spi_rp1_rx_i),
    .spi_clk_o           (spi_rp1_sck_o)
  );

  // SPI host for the Arduino Shield.
  spi u_spi_ard (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    .tl_i                (tl_spi_ard_h2d),
    .tl_o                (tl_spi_ard_d2h),

    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    .spi_copi_o          (spi_ard_tx_o),
    .spi_cipo_i          (spi_ard_rx_i),
    .spi_clk_o           (spi_ard_sck_o)
  );

  // SPI host for mikroBUS Click.
  spi u_spi_mkr (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    .tl_i                (tl_spi_mkr_h2d),
    .tl_o                (tl_spi_mkr_d2h),

    .intr_rx_full_o      (),
    .intr_rx_watermark_o (),
    .intr_tx_empty_o     (),
    .intr_tx_watermark_o (),
    .intr_complete_o     (),

    .spi_copi_o          (spi_mkr_tx_o),
    .spi_cipo_i          (spi_mkr_rx_i),
    .spi_clk_o           (spi_mkr_sck_o)
  );

  // RISC-V timer.
  rv_timer #(
    .DataWidth    ( BusDataWidth ),
    .AddressWidth ( BusAddrWidth )
  ) u_rv_timer (
    .clk_i          (clk_sys_i),
    .rst_ni         (rst_sys_ni),

    // Bus interface.
    .timer_req_i    (device_req[Timer]),
    .timer_we_i     (device_we[Timer]),
    .timer_be_i     (device_be[Timer]),
    .timer_addr_i   (device_addr[Timer]),
    .timer_wdata_i  (device_wdata[Timer]),
    .timer_rvalid_o (device_rvalid[Timer]),
    .timer_rdata_o  (device_rdata[Timer]),
    .timer_err_o    (device_err[Timer]),
    .timer_intr_o   (timer_irq)
  );

  // RISC-V platform level interrupt controller.
  rv_plic u_rv_plic (
    .clk_i      (clk_sys_i),
    .rst_ni     (rst_sys_ni),

    .irq_o      (external_irq),
    .irq_id_o   (),
    .tl_i       (tl_rv_plic_h2d),
    .tl_o       (tl_rv_plic_d2h),

    .intr_src_i (intr_vector)
  );

  // Number of clock cycles in 1.25us. The divide by 10 exists to avoid integer overflow.
  localparam int unsigned RGBLEDCtrlCycleTime = (125 * (SysClkFreq / 10)) / (10_000_000);

  // Controller for multi-colored RGB LEDs.
  rgbled_ctrl #(
    .CycleTime(RGBLEDCtrlCycleTime)
  ) u_rgbled_ctrl(
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .tl_i   (tl_rgbled_ctrl_h2d),
    .tl_o   (tl_rgbled_ctrl_d2h),

    .rgbled_dout_o
  );

  // Debug module top.
  dm_top #(
    .NrHarts      ( 1                              ),
    .IdcodeValue  ( jtag_id_pkg::RV_DM_JTAG_IDCODE )
  ) u_dm_top (
    .clk_i          (clk_sys_i),
    .rst_ni         (rst_sys_ni),
    .testmode_i     (1'b0),
    .ndmreset_o     (ndmreset_req),
    .dmactive_o     (),
    .debug_req_o    (), // TODO connect to debug_req_i
    .unavailable_i  (1'b0),

    // TODO: Bus device with debug memory (for execution-based debug).
    .device_req_i   (0),
    .device_we_i    (0),
    .device_addr_i  (0),
    .device_be_i    (0),
    .device_wdata_i (0),
    .device_rdata_o ( ),

    // Bus host (for system bus accesses, SBA).
    .host_req_o     (host_req[DbgHost]),
    .host_add_o     (host_addr[DbgHost]),
    .host_we_o      (host_we[DbgHost]),
    .host_wdata_o   (host_wdata[DbgHost]),
    .host_be_o      (host_be[DbgHost]),
    .host_gnt_i     (host_gnt[DbgHost]),
    .host_r_valid_i (host_rvalid[DbgHost]),
    .host_r_rdata_i (host_rdata[DbgHost]),

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
    if (i != RevTags) begin
      logic _unused_rvalid;
      assign _unused_rvalid = device_rvalid[i];
    end
  end : gen_unused_device

  logic _unused_be;
  assign _unused_be = |device_be[RevTags];

  logic _unused_tsaddr;
  assign _unused_tsaddr = |tsmap_addr[TsMapAddrWidth-1:RevTagAddrWidth];
endmodule
