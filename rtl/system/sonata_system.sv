// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Sonata system, which instantiates a CHERIoT Ibex, TileLink Uncached
// Lightweight bus and a number of common peripherals, usc as I2C, SPI, UART,
// USB.
module sonata_system #(
  parameter int unsigned GpiWidth      = 13,
  parameter int unsigned GpoWidth      = 24,
  parameter int unsigned ArdAniWidth   = 6,
  parameter int unsigned HalfWordWidth = 16,
  parameter int unsigned WordWidth     = 32,
  parameter int unsigned PwmWidth      = 12,
  parameter int unsigned CheriErrWidth =  9,
  parameter SRAMInitFile               = "",
  parameter int unsigned SysClkFreq    = 30_000_000,
  parameter int unsigned HRClkFreq     = 100_000_000,
  parameter bit DisableHyperram        = 1'b0
) (
  // Main system clock and reset
  input logic                      clk_sys_i,
  input logic                      rst_sys_ni,

  // USB device clock and reset
  input logic                      clk_usb_i,
  input logic                      rst_usb_ni,

  // Hyperram clocks and reset
  input logic                      clk_hr_i,
  input logic                      clk_hr90p_i,
  input logic                      clk_hr3x_i,

  // General purpose input and output
  input  logic [GpiWidth-1:0]      gp_i,
  output logic [GpoWidth-1:0]      gp_o,
  // 0: Raspberry Pi HAT g0-15
  // 1: Raspberry Pi HAT g16-27
  // 2: Arduino Shield 0-15
  // 3: Arduino Shield 16-17
  // 4: PMOD
  input  logic [HalfWordWidth-1:0] gp_headers_i [sonata_pkg::GPIO_NUM],
  output logic [WordWidth-1:0]     gp_headers_o [sonata_pkg::GPIO_NUM],

  output logic [PwmWidth-1:0]      pwm_o,

  // Arduino shield analog(ue) inputs:
  // Digital version of inputs, then p & n true analog(ue) inputs
  input  logic [ArdAniWidth-1:0]   ard_an_di_i,
  input  wire  [ArdAniWidth-1:0]   ard_an_p_i,
  input  wire  [ArdAniWidth-1:0]   ard_an_n_i,

  // UARTs
  //   0 and 1: FTDI chip
  //   2: Raspberry Pi HAT
  //   3: mikroBUS Click
  //   4: RS-232
  input  logic                     uart_rx_i [sonata_pkg::UART_NUM],
  output logic                     uart_tx_o [sonata_pkg::UART_NUM],

  // I2C buses
  input  logic                     i2c_scl_i    [sonata_pkg::I2C_NUM],
  output logic                     i2c_scl_o    [sonata_pkg::I2C_NUM],
  output logic                     i2c_scl_en_o [sonata_pkg::I2C_NUM],
  input  logic                     i2c_sda_i    [sonata_pkg::I2C_NUM],
  output logic                     i2c_sda_o    [sonata_pkg::I2C_NUM],
  output logic                     i2c_sda_en_o [sonata_pkg::I2C_NUM],

  // SPI hosts
  //   0: flash
  //   1: LCD
  //   2: ethernet
  //   3: Raspberry Pi HAT SPI0
  //   4: Raspberry Pi HAT SPI1
  //   5: Arduino Shield
  //   6: mikroBUS Click
  input  logic                     spi_rx_i  [sonata_pkg::SPI_NUM],
  output logic                     spi_tx_o  [sonata_pkg::SPI_NUM],
  output logic                     spi_sck_o [sonata_pkg::SPI_NUM],

  input  logic                     spi_eth_irq_ni, // Interrupt from Ethernet MAC

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

  output logic                     rgbled_dout_o,

  inout  wire [7:0]                hyperram_dq,
  inout  wire                      hyperram_rwds,
  output wire                      hyperram_ckp,
  output wire                      hyperram_ckn,
  output wire                      hyperram_nrst,
  output wire                      hyperram_cs,

  output tlul_pkg::tl_h2d_t        tl_pinmux_o,
  input  tlul_pkg::tl_d2h_t        tl_pinmux_i
);

  import sonata_pkg::*;

  ///////////////////////////////////////////////
  // Signals, types and parameters for system. //
  ///////////////////////////////////////////////

  localparam int unsigned MemSize       = DisableHyperram ? 256 * 1024 : 128 * 1024; // 256 KiB
  localparam int unsigned SRAMAddrWidth = $clog2(MemSize);
  localparam int unsigned HyperRAMSize  = 1024 * 1024; // 1 MiB
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
    Pwm,
    Timer,
    RevTags,
    HwRev
  } bus_device_e;

  localparam int NrNonGpioDevices = 5;
  localparam int NrDevices = NrNonGpioDevices + GPIO_NUM;
  localparam int NrHosts = 2;

  // Signals for hardware revoker
  logic [127:0] hardware_revoker_control_reg_rdata;
  logic [63:0]  hardware_revoker_control_reg_wdata;
  logic         hardware_revoker_irq;

  // Interrupts.
  logic timer_irq;
  logic external_irq;

  logic uart_tx_watermark_irq [UART_NUM];
  logic uart_rx_watermark_irq [UART_NUM];
  logic uart_tx_empty_irq     [UART_NUM];
  logic uart_rx_overflow_irq  [UART_NUM];
  logic uart_rx_frame_err_irq [UART_NUM];
  logic uart_rx_break_err_irq [UART_NUM];
  logic uart_rx_timeout_irq   [UART_NUM];
  logic uart_rx_parity_err_irq[UART_NUM];

  logic i2c_fmt_threshold_irq   [I2C_NUM];
  logic i2c_rx_threshold_irq    [I2C_NUM];
  logic i2c_acq_threshold_irq   [I2C_NUM];
  logic i2c_rx_overflow_irq     [I2C_NUM];
  logic i2c_nak_irq             [I2C_NUM];
  logic i2c_scl_interference_irq[I2C_NUM];
  logic i2c_sda_interference_irq[I2C_NUM];
  logic i2c_stretch_timeout_irq [I2C_NUM];
  logic i2c_sda_unstable_irq    [I2C_NUM];
  logic i2c_cmd_complete_irq    [I2C_NUM];
  logic i2c_tx_stretch_irq      [I2C_NUM];
  logic i2c_tx_threshold_irq    [I2C_NUM];
  logic i2c_acq_full_irq        [I2C_NUM];
  logic i2c_unexp_stop_irq      [I2C_NUM];
  logic i2c_host_timeout_irq    [I2C_NUM];

  logic spi_eth_irq;

  logic usbdev_pkt_received_irq;
  logic usbdev_pkt_sent_irq;
  logic usbdev_powered_irq;
  logic usbdev_disconnected_irq;
  logic usbdev_host_lost_irq;
  logic usbdev_link_reset_irq;
  logic usbdev_link_suspend_irq;
  logic usbdev_link_resume_irq;
  logic usbdev_av_out_empty_irq;
  logic usbdev_rx_full_irq;
  logic usbdev_av_overflow_irq;
  logic usbdev_link_in_err_irq;
  logic usbdev_link_out_err_irq;
  logic usbdev_rx_crc_err_irq;
  logic usbdev_rx_pid_err_irq;
  logic usbdev_rx_bitstuff_err_irq;
  logic usbdev_frame_irq;
  logic usbdev_av_setup_empty_irq;

  logic [181:0] intr_vector;

  localparam int unsigned UartIrqs = 8;
  localparam int unsigned I2cIrqs  = 15;
  localparam int unsigned ExtraUarts = UART_NUM > 5 ? UART_NUM - 5 : 0;
  localparam int unsigned ExtraI2cs  =  I2C_NUM > 2 ?  I2C_NUM - 2 : 0;
  assign intr_vector[181 : (100 + UartIrqs*ExtraUarts + I2cIrqs*ExtraI2cs)] = '0;

  for (genvar i = 0; i < ExtraI2cs; i++) begin : gen_i2c_intr_1
    assign intr_vector[(114 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_host_timeout_irq    [i];
    assign intr_vector[(113 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_unexp_stop_irq      [i];
    assign intr_vector[(112 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_acq_full_irq        [i];
    assign intr_vector[(111 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_tx_threshold_irq    [i];
    assign intr_vector[(110 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_tx_stretch_irq      [i];
    assign intr_vector[(109 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_cmd_complete_irq    [i];
    assign intr_vector[(108 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_sda_unstable_irq    [i];
    assign intr_vector[(107 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_stretch_timeout_irq [i];
    assign intr_vector[(106 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_sda_interference_irq[i];
    assign intr_vector[(105 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_scl_interference_irq[i];
    assign intr_vector[(104 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_nak_irq             [i];
    assign intr_vector[(103 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_rx_overflow_irq     [i];
    assign intr_vector[(102 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_acq_threshold_irq   [i];
    assign intr_vector[(101 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_rx_threshold_irq    [i];
    assign intr_vector[(100 + i*I2cIrqs + ExtraUarts*UartIrqs) +: 1] = i2c_fmt_threshold_irq   [i];
  end : gen_i2c_intr_1

  for (genvar i = 0; i < ExtraUarts; i++) begin : gen_uart_intr_2
    assign intr_vector[(107 + i*UartIrqs) +: 1] = uart_rx_parity_err_irq[i];
    assign intr_vector[(106 + i*UartIrqs) +: 1] = uart_rx_timeout_irq   [i];
    assign intr_vector[(105 + i*UartIrqs) +: 1] = uart_rx_break_err_irq [i];
    assign intr_vector[(104 + i*UartIrqs) +: 1] = uart_rx_frame_err_irq [i];
    assign intr_vector[(103 + i*UartIrqs) +: 1] = uart_rx_overflow_irq  [i];
    assign intr_vector[(102 + i*UartIrqs) +: 1] = uart_tx_empty_irq     [i];
    assign intr_vector[(101 + i*UartIrqs) +: 1] = uart_rx_watermark_irq [i];
    assign intr_vector[(100 + i*UartIrqs) +: 1] = uart_tx_watermark_irq [i];
  end : gen_uart_intr_2

  assign intr_vector[99 +: 1] = usbdev_av_setup_empty_irq;
  assign intr_vector[98 +: 1] = usbdev_frame_irq;
  assign intr_vector[97 +: 1] = usbdev_rx_bitstuff_err_irq;
  assign intr_vector[96 +: 1] = usbdev_rx_pid_err_irq;
  assign intr_vector[95 +: 1] = usbdev_rx_crc_err_irq;
  assign intr_vector[94 +: 1] = usbdev_link_out_err_irq;
  assign intr_vector[93 +: 1] = usbdev_link_in_err_irq;
  assign intr_vector[92 +: 1] = usbdev_av_overflow_irq;
  assign intr_vector[91 +: 1] = usbdev_rx_full_irq;
  assign intr_vector[90 +: 1] = usbdev_av_out_empty_irq;
  assign intr_vector[89 +: 1] = usbdev_link_resume_irq;
  assign intr_vector[88 +: 1] = usbdev_link_suspend_irq;
  assign intr_vector[87 +: 1] = usbdev_link_reset_irq;
  assign intr_vector[86 +: 1] = usbdev_host_lost_irq;
  assign intr_vector[85 +: 1] = usbdev_disconnected_irq;
  assign intr_vector[84 +: 1] = usbdev_powered_irq;
  assign intr_vector[83 +: 1] = usbdev_pkt_sent_irq;
  assign intr_vector[82 +: 1] = usbdev_pkt_received_irq;

  // Reserved for future use.
  assign intr_vector[73 +: 9] = 9'b0;

  assign intr_vector[72 +: 1] = hardware_revoker_irq;

  for (genvar i = 2; i < 5; i++) begin : gen_uart_intr_1
    assign intr_vector[(55 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_parity_err_irq[i] : 1'b0;
    assign intr_vector[(54 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_timeout_irq   [i] : 1'b0;
    assign intr_vector[(53 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_break_err_irq [i] : 1'b0;
    assign intr_vector[(52 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_frame_err_irq [i] : 1'b0;
    assign intr_vector[(51 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_overflow_irq  [i] : 1'b0;
    assign intr_vector[(50 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_tx_empty_irq     [i] : 1'b0;
    assign intr_vector[(49 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_watermark_irq [i] : 1'b0;
    assign intr_vector[(48 + (i-2)*UartIrqs) +: 1] = i < UART_NUM ? uart_tx_watermark_irq [i] : 1'b0;
  end : gen_uart_intr_1

  assign intr_vector[47 +: 1] = spi_eth_irq;

  for (genvar i = 0; i < 2; i++) begin : gen_i2c_intr_0
    assign intr_vector[(31 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_host_timeout_irq    [i] : 1'b0;
    assign intr_vector[(30 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_unexp_stop_irq      [i] : 1'b0;
    assign intr_vector[(29 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_acq_full_irq        [i] : 1'b0;
    assign intr_vector[(28 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_tx_threshold_irq    [i] : 1'b0;
    assign intr_vector[(27 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_tx_stretch_irq      [i] : 1'b0;
    assign intr_vector[(26 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_cmd_complete_irq    [i] : 1'b0;
    assign intr_vector[(25 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_sda_unstable_irq    [i] : 1'b0;
    assign intr_vector[(24 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_stretch_timeout_irq [i] : 1'b0;
    assign intr_vector[(23 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_sda_interference_irq[i] : 1'b0;
    assign intr_vector[(22 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_scl_interference_irq[i] : 1'b0;
    assign intr_vector[(21 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_nak_irq             [i] : 1'b0;
    assign intr_vector[(20 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_rx_overflow_irq     [i] : 1'b0;
    assign intr_vector[(19 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_acq_threshold_irq   [i] : 1'b0;
    assign intr_vector[(18 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_rx_threshold_irq    [i] : 1'b0;
    assign intr_vector[(17 + i*I2cIrqs) +: 1] = i < I2C_NUM ? i2c_fmt_threshold_irq   [i] : 1'b0;
  end : gen_i2c_intr_0

  for (genvar i = 0; i < 2; i++) begin : gen_uart_intr_0
    assign intr_vector[( 8 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_parity_err_irq[i] : 1'b0;
    assign intr_vector[( 7 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_timeout_irq   [i] : 1'b0;
    assign intr_vector[( 6 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_break_err_irq [i] : 1'b0;
    assign intr_vector[( 5 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_frame_err_irq [i] : 1'b0;
    assign intr_vector[( 4 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_overflow_irq  [i] : 1'b0;
    assign intr_vector[( 3 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_tx_empty_irq     [i] : 1'b0;
    assign intr_vector[( 2 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_rx_watermark_irq [i] : 1'b0;
    assign intr_vector[( 1 + i*UartIrqs) +: 1] = i < UART_NUM ? uart_tx_watermark_irq [i] : 1'b0;
  end : gen_uart_intr_0

  assign intr_vector[ 0 +: 1] = 1'b0; // This is a special case and tied to zero.

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
  assign device_req[Pwm]      = device_re[Pwm]      | device_we[Pwm];
  assign device_req[Timer]    = device_re[Timer]    | device_we[Timer];
  assign device_req[HwRev]    = device_re[HwRev]    | device_we[HwRev];

  // Instruction fetch signals.
  logic                    core_instr_req;
  logic                    core_instr_gnt;
  logic                    core_instr_rvalid;
  logic [BusAddrWidth-1:0] core_instr_addr;
  logic [BusDataWidth-1:0] core_instr_rdata;
  logic                    core_instr_err;

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
  assign device_err[Pwm]      = 1'b0;
  assign device_err[HwRev]    = 1'b0;

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
  tlul_pkg::tl_h2d_t tl_sram_a_h2d_d;
  tlul_pkg::tl_d2h_t tl_sram_a_d2h_d;
  tlul_pkg::tl_h2d_t tl_sram_a_h2d_q;
  tlul_pkg::tl_d2h_t tl_sram_a_d2h_q;
  tlul_pkg::tl_h2d_t tl_sram_b_h2d;
  tlul_pkg::tl_d2h_t tl_sram_b_d2h;
  tlul_pkg::tl_h2d_t tl_hyperram_us_h2d[2];
  tlul_pkg::tl_d2h_t tl_hyperram_us_d2h[2];
  tlul_pkg::tl_h2d_t tl_hyperram_ds_h2d;
  tlul_pkg::tl_d2h_t tl_hyperram_ds_d2h;
  tlul_pkg::tl_h2d_t tl_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_xadc_h2d;
  tlul_pkg::tl_d2h_t tl_xadc_d2h;
  tlul_pkg::tl_h2d_t tl_gpio_headers_h2d[GPIO_NUM];
  tlul_pkg::tl_d2h_t tl_gpio_headers_d2h[GPIO_NUM];
  tlul_pkg::tl_h2d_t tl_uart_h2d[UART_NUM];
  tlul_pkg::tl_d2h_t tl_uart_d2h[UART_NUM];
  tlul_pkg::tl_h2d_t tl_timer_h2d;
  tlul_pkg::tl_d2h_t tl_timer_d2h;
  tlul_pkg::tl_h2d_t tl_rgbled_ctrl_h2d;
  tlul_pkg::tl_d2h_t tl_rgbled_ctrl_d2h;
  tlul_pkg::tl_h2d_t tl_pwm_h2d;
  tlul_pkg::tl_d2h_t tl_pwm_d2h;
  tlul_pkg::tl_h2d_t tl_i2c_h2d[I2C_NUM];
  tlul_pkg::tl_d2h_t tl_i2c_d2h[I2C_NUM];
  tlul_pkg::tl_h2d_t tl_rv_plic_h2d;
  tlul_pkg::tl_d2h_t tl_rv_plic_d2h;
  tlul_pkg::tl_h2d_t tl_spi_h2d[SPI_NUM];
  tlul_pkg::tl_d2h_t tl_spi_d2h[SPI_NUM];
  tlul_pkg::tl_h2d_t tl_usbdev_h2d;
  tlul_pkg::tl_d2h_t tl_usbdev_d2h;
  tlul_pkg::tl_h2d_t tl_rev_tag_h2d;
  tlul_pkg::tl_d2h_t tl_rev_tag_d2h;
  tlul_pkg::tl_h2d_t tl_hw_rev_h2d;
  tlul_pkg::tl_d2h_t tl_hw_rev_d2h;

  sonata_xbar_main xbar (
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
    .tl_sram_o        (tl_sram_a_h2d_d),
    .tl_sram_i        (tl_sram_a_d2h_d),
    .tl_hyperram_o    (tl_hyperram_us_h2d[0]),
    .tl_hyperram_i    (tl_hyperram_us_d2h[0]),
    .tl_rev_tag_o     (tl_rev_tag_h2d),
    .tl_rev_tag_i     (tl_rev_tag_d2h),
    .tl_gpio_o        (tl_gpio_h2d),
    .tl_gpio_i        (tl_gpio_d2h),
    .tl_pwm_o         (tl_pwm_h2d),
    .tl_pwm_i         (tl_pwm_d2h),
    .tl_pinmux_o      (tl_pinmux_o),
    .tl_pinmux_i      (tl_pinmux_i),
    .tl_gpio_headers_o(tl_gpio_headers_h2d),
    .tl_gpio_headers_i(tl_gpio_headers_d2h),
    .tl_rgbled_ctrl_o (tl_rgbled_ctrl_h2d),
    .tl_rgbled_ctrl_i (tl_rgbled_ctrl_d2h),
    .tl_hw_rev_o      (tl_hw_rev_h2d),
    .tl_hw_rev_i      (tl_hw_rev_d2h),
    .tl_xadc_o        (tl_xadc_h2d),
    .tl_xadc_i        (tl_xadc_d2h),
    .tl_timer_o       (tl_timer_h2d),
    .tl_timer_i       (tl_timer_d2h),
    .tl_uart_o        (tl_uart_h2d),
    .tl_uart_i        (tl_uart_d2h),
    .tl_i2c_o         (tl_i2c_h2d),
    .tl_i2c_i         (tl_i2c_d2h),
    .tl_spi_o         (tl_spi_h2d),
    .tl_spi_i         (tl_spi_d2h),
    .tl_usbdev_o      (tl_usbdev_h2d),
    .tl_usbdev_i      (tl_usbdev_d2h),
    .tl_rv_plic_o     (tl_rv_plic_h2d),
    .tl_rv_plic_i     (tl_rv_plic_d2h)
  );

  xbar_ifetch u_xbar_ifetch (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),
    .tl_ibex_ifetch_i (tl_ibex_ins_h2d),
    .tl_ibex_ifetch_o (tl_ibex_ins_d2h),

    .tl_sram_o     (tl_sram_b_h2d),
    .tl_sram_i     (tl_sram_b_d2h),
    .tl_hyperram_o (tl_hyperram_us_h2d[1]),
    .tl_hyperram_i (tl_hyperram_us_d2h[1]),

    .scanmode_i (prim_mubi_pkg::MuBi4False)
  );

  // TL-UL host adapter(s).

  tlul_adapter_host ibex_ins_host_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    .req_i        (core_instr_req),
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

    .tl_h_i      (tl_sram_a_h2d_d),
    .tl_h_o      (tl_sram_a_d2h_d),
    .tl_d_o      (tl_sram_a_h2d_q),
    .tl_d_i      (tl_sram_a_d2h_q),

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

    .tl_a_i (tl_sram_a_h2d_q),
    .tl_a_o (tl_sram_a_d2h_q),
    .tl_b_i (tl_sram_b_h2d),
    .tl_b_o (tl_sram_b_d2h)
  );

  if (DisableHyperram) begin : g_no_hyperram
    logic unused_clk_hr;
    logic unused_clk_hr90p;
    logic unused_clk_hr3x;

    assign unused_clk_hr    = clk_hr_i;
    assign unused_clk_hr90p = clk_hr90p_i;
    assign unused_clk_hr3x  = clk_hr3x_i;

    assign hyperram_dq   = '0;
    assign hyperram_rwds = '0;
    assign hyperram_ckp  = 1'b0;
    assign hyperram_ckn  = 1'b0;
    assign hyperram_nrst = 1'b0;
    assign hyperram_cs   = 1'b0;

    tlul_err_resp u_hyperram_err (
      .clk_i (clk_sys_i),
      .rst_ni (rst_sys_ni),
      .tl_h_i(tl_hyperram_ds_h2d),
      .tl_h_o(tl_hyperram_ds_d2h)
    );
  end else begin : g_hyperram
    hyperram #(
      .HRClkFreq   (HRClkFreq),
      .HyperRAMSize(HyperRAMSize)
    ) u_hyperram (
      .clk_i  (clk_sys_i),
      .rst_ni (rst_sys_ni),

      .clk_hr_i,
      .clk_hr90p_i,
      .clk_hr3x_i,

      .tl_i (tl_hyperram_ds_h2d),
      .tl_o (tl_hyperram_ds_d2h),

      .hyperram_dq,
      .hyperram_rwds,
      .hyperram_ckp,
      .hyperram_ckn,
      .hyperram_nrst,
      .hyperram_cs
    );
  end

  // Manual M:1 socket instantiation as xbar generator cannot deal with multiple ports for one
  // device and we want to utilize the dual port SRAM. So totally separate crossbars are generated
  // for the dside and iside then tlul_socket_m1 is used here to connect the two crossbars to the
  // one downstream hyperram tilelink port.
  //
  // US == Upstream
  // DS == Downstream
  //
  // US is the Ibex/Host end, DS is the Hyperram end.
  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_hyperram_tl_socket (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),
    .tl_h_i(tl_hyperram_us_h2d),
    .tl_h_o(tl_hyperram_us_d2h),
    .tl_d_o(tl_hyperram_ds_h2d),
    .tl_d_i(tl_hyperram_ds_d2h)
  );

  tlul_adapter_reg #(
    .EnableRspIntgGen ( 1 ),
    .AccessLatency    ( 1 )
  ) hardware_revoker_control_reg_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_hw_rev_h2d),
    .tl_o         (tl_hw_rev_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[HwRev]),
    .we_o         (device_we[HwRev]),
    .addr_o       (device_addr[HwRev][RegAddrWidth-1:0]),
    .wdata_o      (device_wdata[HwRev]),
    .be_o         (device_be[HwRev]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[HwRev]),
    .error_i      (device_err[HwRev])
  );

  // Tie off upper bits of address.
  assign device_addr[HwRev][BusAddrWidth-1:RegAddrWidth] = '0;

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
    .tl_i         (tl_rev_tag_h2d),
    .tl_o         (tl_rev_tag_d2h),

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
    .RV32B           ( ibex_pkg::RV32BFull                     ),
    .ICache          ( 1'b1                                    )
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

    .mmreg_corein_i         (hardware_revoker_control_reg_rdata),
    .mmreg_coreout_o        (hardware_revoker_control_reg_wdata),
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

  msftDvIp_mmreg hardware_revoker_control_reg (
    .clk_i           (clk_sys_i),
    .rstn_i          (rst_core_n),

    .reg_en_i        (device_req[HwRev]),
    .reg_addr_i      (device_addr[HwRev]),
    .reg_wdata_i     (device_wdata[HwRev]),
    .reg_we_i        (device_we[HwRev]),
    .reg_rdata_o     (device_rdata[HwRev]),
    .reg_ready_o     (),

    .mmreg_coreout_i (hardware_revoker_control_reg_wdata),
    .mmreg_corein_o  (hardware_revoker_control_reg_rdata),
    .tbre_intr_o     (hardware_revoker_irq)
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
    .gp_o,
    .gp_o_en()
  );

  for (genvar i = 0; i < sonata_pkg::GPIO_NUM; i++) begin : gen_gpio_headers
    tlul_adapter_reg #(
      .EnableRspIntgGen ( 1 ),
      .AccessLatency    ( 1 )
    ) gpio_device_adapter (
      .clk_i        (clk_sys_i),
      .rst_ni       (rst_sys_ni),

      // TL-UL interface.
      .tl_i         (tl_gpio_headers_h2d[i]),
      .tl_o         (tl_gpio_headers_d2h[i]),

      // Control interface.
      .en_ifetch_i  (prim_mubi_pkg::MuBi4False),
      .intg_error_o (),

      // Register interface.
      .re_o         (device_re   [NrNonGpioDevices+i]),
      .we_o         (device_we   [NrNonGpioDevices+i]),
      .addr_o       (device_addr [NrNonGpioDevices+i][RegAddrWidth-1:0]),
      .wdata_o      (device_wdata[NrNonGpioDevices+i]),
      .be_o         (device_be   [NrNonGpioDevices+i]),
      .busy_i       ('0),
      .rdata_i      (device_rdata[NrNonGpioDevices+i]),
      .error_i      (device_err  [NrNonGpioDevices+i])
    );

    assign device_req [NrNonGpioDevices+i] = device_re[NrNonGpioDevices+i] | device_we[NrNonGpioDevices+i];
    assign device_err [NrNonGpioDevices+i] = 1'b0;
    assign device_addr[NrNonGpioDevices+i][BusAddrWidth-1:RegAddrWidth] = '0;

    gpio #(
      .GpiWidth ( HalfWordWidth ),
      .GpoWidth ( WordWidth     )
    ) u_gpio (
      .clk_i           (clk_sys_i),
      .rst_ni          (rst_sys_ni),

      // Bus interface.
      .device_req_i    (device_req   [NrNonGpioDevices+i]),
      .device_addr_i   (device_addr  [NrNonGpioDevices+i]),
      .device_we_i     (device_we    [NrNonGpioDevices+i]),
      .device_be_i     (device_be    [NrNonGpioDevices+i]),
      .device_wdata_i  (device_wdata [NrNonGpioDevices+i]),
      .device_rvalid_o (device_rvalid[NrNonGpioDevices+i]),
      .device_rdata_o  (device_rdata [NrNonGpioDevices+i]),

      .gp_i            (gp_headers_i[i]),
      .gp_o            (gp_headers_o[i]),
      .gp_o_en         ()
    );

  end : gen_gpio_headers

  // Digital inputs from Arduino sheild analog(ue) pins currently unused
  logic unused_ard_an_di;
  assign unused_ard_an_di = ^ard_an_di_i;

  // XADC - Xilinx Hard-IP Analog(ue) to Digital Converter
  xadc u_xadc(
    .clk_i     (clk_sys_i),
    .rst_ni    (rst_sys_ni),

    .tl_i      (tl_xadc_h2d),
    .tl_o      (tl_xadc_d2h),

    .analog_p_i(ard_an_p_i),
    .analog_n_i(ard_an_n_i)
  );

  for (genvar i = 0; i < I2C_NUM; i++) begin : gen_i2c_hosts
    i2c u_i2c (
      .clk_i                   (clk_sys_i),
      .rst_ni                  (rst_sys_ni),
      .ram_cfg_i               (10'b0),

      // Bus interface.
      .tl_i                    (tl_i2c_h2d[i]),
      .tl_o                    (tl_i2c_d2h[i]),

      // Generic IO.
      .cio_scl_i               (i2c_scl_i   [i]),
      .cio_scl_o               (i2c_scl_o   [i]),
      .cio_scl_en_o            (i2c_scl_en_o[i]),
      .cio_sda_i               (i2c_sda_i   [i]),
      .cio_sda_o               (i2c_sda_o   [i]),
      .cio_sda_en_o            (i2c_sda_en_o[i]),

      // Interrupts.
      .intr_fmt_threshold_o    (i2c_fmt_threshold_irq   [i]),
      .intr_rx_threshold_o     (i2c_rx_threshold_irq    [i]),
      .intr_acq_threshold_o    (i2c_acq_threshold_irq   [i]),
      .intr_rx_overflow_o      (i2c_rx_overflow_irq     [i]),
      .intr_nak_o              (i2c_nak_irq             [i]),
      .intr_scl_interference_o (i2c_scl_interference_irq[i]),
      .intr_sda_interference_o (i2c_sda_interference_irq[i]),
      .intr_stretch_timeout_o  (i2c_stretch_timeout_irq [i]),
      .intr_sda_unstable_o     (i2c_sda_unstable_irq    [i]),
      .intr_cmd_complete_o     (i2c_cmd_complete_irq    [i]),
      .intr_tx_stretch_o       (i2c_tx_stretch_irq      [i]),
      .intr_tx_threshold_o     (i2c_tx_threshold_irq    [i]),
      .intr_acq_full_o         (i2c_acq_full_irq        [i]),
      .intr_unexp_stop_o       (i2c_unexp_stop_irq      [i]),
      .intr_host_timeout_o     (i2c_host_timeout_irq    [i])
    );
  end : gen_i2c_hosts

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

  // UART for serial communication.
  for (genvar i = 0; i < UART_NUM; i++) begin : gen_uart_blocks
    uart u_uart (
      .clk_i                (clk_sys_i),
      .rst_ni               (rst_sys_ni),

      .cio_rx_i             (uart_rx_i[i]),
      .cio_tx_o             (uart_tx_o[i]),
      .cio_tx_en_o          (),

      // Inter-module signals.
      .tl_i                 (tl_uart_h2d[i]),
      .tl_o                 (tl_uart_d2h[i]),

      // Interrupts.
      .intr_tx_watermark_o  (uart_tx_watermark_irq [i]),
      .intr_rx_watermark_o  (uart_rx_watermark_irq [i]),
      .intr_tx_empty_o      (uart_tx_empty_irq     [i]),
      .intr_rx_overflow_o   (uart_rx_overflow_irq  [i]),
      .intr_rx_frame_err_o  (uart_rx_frame_err_irq [i]),
      .intr_rx_break_err_o  (uart_rx_break_err_irq [i]),
      .intr_rx_timeout_o    (uart_rx_timeout_irq   [i]),
      .intr_rx_parity_err_o (uart_rx_parity_err_irq[i])
    );
  end : gen_uart_blocks

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
    .intr_pkt_received_o          (usbdev_pkt_received_irq),
    .intr_pkt_sent_o              (usbdev_pkt_sent_irq),
    .intr_powered_o               (usbdev_powered_irq),
    .intr_disconnected_o          (usbdev_disconnected_irq),
    .intr_host_lost_o             (usbdev_host_lost_irq),
    .intr_link_reset_o            (usbdev_link_reset_irq),
    .intr_link_suspend_o          (usbdev_link_suspend_irq),
    .intr_link_resume_o           (usbdev_link_resume_irq),
    .intr_av_out_empty_o          (usbdev_av_out_empty_irq),
    .intr_rx_full_o               (usbdev_rx_full_irq),
    .intr_av_overflow_o           (usbdev_av_overflow_irq),
    .intr_link_in_err_o           (usbdev_link_in_err_irq),
    .intr_link_out_err_o          (usbdev_link_out_err_irq),
    .intr_rx_crc_err_o            (usbdev_rx_crc_err_irq),
    .intr_rx_pid_err_o            (usbdev_rx_pid_err_irq),
    .intr_rx_bitstuff_err_o       (usbdev_rx_bitstuff_err_irq),
    .intr_frame_o                 (usbdev_frame_irq),
    .intr_av_setup_empty_o        (usbdev_av_setup_empty_irq)
  );

  // SPI hosts.
  for (genvar i = 0; i < SPI_NUM; i++) begin : gen_spi_hosts
    spi u_spi (
      .clk_i               (clk_sys_i),
      .rst_ni              (rst_sys_ni),

      // TileLink interface.
      .tl_i                (tl_spi_h2d[i]),
      .tl_o                (tl_spi_d2h[i]),

      // Interrupts currently disconnected.
      .intr_rx_full_o      (),
      .intr_rx_watermark_o (),
      .intr_tx_empty_o     (),
      .intr_tx_watermark_o (),
      .intr_complete_o     (),

      // SPI signals.
      .spi_copi_o          (spi_tx_o [i]),
      .spi_cipo_i          (spi_rx_i [i]),
      .spi_clk_o           (spi_sck_o[i])
    );
  end : gen_spi_hosts

  // Sample the ethernet interrupt pin.
  always_ff @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      spi_eth_irq <= 1'b0;
    end else begin
      spi_eth_irq <= !spi_eth_irq_ni;
    end
  end

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
