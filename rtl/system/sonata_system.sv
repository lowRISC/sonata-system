// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// The Sonata system, which instantiates a CHERIoT Ibex, TileLink Uncached
// Lightweight bus and a number of common peripherals, usc as I2C, SPI, UART,
// USB.
module sonata_system
  import sonata_pkg::*;
#(
  parameter int unsigned ArdAniWidth     = 6,
  parameter int unsigned CheriErrWidth   = 9,
  parameter string SRAMInitFile          = "",
  parameter int unsigned SysClkFreq      = 30_000_000,
  parameter int unsigned HyperRAMClkFreq = 100_000_000
) (
  // Main system clock and reset
  input logic                      clk_sys_i,
  input logic                      rst_sys_ni,

  // USB device clock and reset
  input logic                      clk_usb_i,
  input logic                      rst_usb_ni,

  // HyperRAM clocks and reset
`ifdef TARGET_XL_BOARD
  // No HyperRAM on Sonata XL
`else
  input logic                      clk_hr_i,
  input logic                      clk_hr90p_i,
  input logic                      clk_hr3x_i,
  input logic                      rst_hr_ni,
`endif

  // General purpose input and output
  input  logic [GPIO_IOS_WIDTH-1:0] gp_i,
  output logic [GPIO_IOS_WIDTH-1:0] gp_o,
  output logic [GPIO_IOS_WIDTH-1:0] gp_o_en,

  // Arduino shield analog(ue) inputs:
  // Digital version of inputs, then p & n true analog(ue) inputs
  input  logic [ArdAniWidth-1:0]   ard_an_di_i,
  input  wire  [ArdAniWidth-1:0]   ard_an_p_i,
  input  wire  [ArdAniWidth-1:0]   ard_an_n_i,

  // Non-pinmuxed spi devices
  output logic                     lcd_copi_o,
  output logic                     lcd_copi_en_o,
  input  logic                     lcd_cipo_i,
  output logic                     lcd_sclk_o,
  output logic                     lcd_cs_o,
  output logic                     lcd_dc_o,
  output logic                     lcd_rst_o,
  output logic                     lcd_backlight_o,

  output logic                     ethmac_copi_o,
  input  logic                     ethmac_cipo_i,
  output logic                     ethmac_sclk_o,
  output logic                     ethmac_cs_o,
  output logic                     ethmac_rst_o,
  input  logic                     ethmac_irq_ni, // Interrupt from Ethernet MAC

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

`ifdef TARGET_XL_BOARD
  // No HyperRAM on Sonata XL
`else
  inout  wire [7:0]                hyperram_dq,
  inout  wire                      hyperram_rwds,
  output wire                      hyperram_ckp,
  output wire                      hyperram_ckn,
  output wire                      hyperram_nrst,
  output wire                      hyperram_cs,
`endif

  output wire                      rs485_rx_enable_o,
  output wire                      rs485_tx_enable_o,

  // Pin Signals
  input  sonata_in_pins_t    in_from_pins_i,
  output sonata_out_pins_t   out_to_pins_o,
  input  sonata_inout_pins_t inout_from_pins_i,
  output sonata_inout_pins_t inout_to_pins_o,
  output sonata_inout_pins_t inout_to_pins_en_o
);
  ///////////////////////////////////////////////
  // Signals, types and parameters for system. //
  ///////////////////////////////////////////////

  localparam int unsigned MemSize       = 128 * 1024; // 128 KiB
  localparam int unsigned SRAMAddrWidth = $clog2(MemSize);
  localparam int unsigned HyperRAMSize  = 1024 * 1024; // 1 MiB
  localparam int unsigned PwmCtrSize    = 8;
  localparam int unsigned BusAddrWidth  = 32;
  localparam int unsigned BusByteEnable = 4;
  localparam int unsigned BusDataWidth  = 32;
  localparam int unsigned DRegAddrWidth = 12; // Debug module uses 12 bits of addressing.
  localparam int unsigned TRegAddrWidth = 16; // Timer uses more address bits.
  localparam int unsigned FixedSpiNum   = 2; // Number of SPI devices that don't pass through the pinmux
  localparam int unsigned TotalSpiNum   = SPI_NUM + FixedSpiNum; // The total number of SPI devices
  localparam int unsigned FixedGpioNum  = 1; // Number of GPIO instances that don't pass through the pinmux
  localparam int unsigned TotalGpioNum  = GPIO_NUM + FixedGpioNum; // The total number of GPIO instances
  localparam int unsigned TAccessLatency = 0; // Cycles of read data latency.

  // The number of data bits controlled by each mask bit; since the CPU requires
  // only byte level access, explicitly grouping the data bits makes the inferred
  // BRAM implementations in FPGA much more efficient.
  localparam int unsigned DataBitsPerMask = BusDataWidth / BusByteEnable;

  // Debug functionality is enabled.
  localparam int unsigned DbgHwBreakNum = 2;
  localparam bit          DbgTriggerEn  = 1'b1;

  typedef enum int {
    CoreD,
    DbgHost
  } bus_host_e;

  typedef enum int {
    Timer,
    RevTags,
    DbgDev
  } bus_device_e;

  localparam int NrDevices = 3;
  localparam int NrHosts = 2;

  // Signals for hardware revoker
  logic [127:0] hardware_revoker_control_reg_rdata;
  logic [63:0]  hardware_revoker_control_reg_wdata;
  logic         hardware_revoker_irq;

  // Interrupts.
  localparam int unsigned I2cIrqs    = 15;
  localparam int unsigned SpiIrqs    = 5;
  localparam int unsigned UartIrqs   = 9;
  localparam int unsigned UsbdevIrqs = 18;

  logic timer_irq;
  logic external_irq;

  logic [I2cIrqs-1:0]    i2c_interrupts [I2C_NUM];
  logic [SpiIrqs-1:0]    spi_interrupts [TotalSpiNum];
  logic [UartIrqs-1:0]   uart_interrupts[UART_NUM];
  logic [UsbdevIrqs-1:0] usbdev_interrupts;
  logic                  gpio_interrupts[TotalGpioNum];

  logic ethmac_irq;

  // Each IP block has a single interrupt line to the PLIC and software shall consult the intr_state
  // register within the block itself to identify the interrupt source(s).
  logic [I2C_NUM-1:0]  i2c_irq;
  logic [TotalSpiNum-1:0]  spi_irq;
  logic [UART_NUM-1:0] uart_irq;
  logic                usbdev_irq;
  logic                gpio_irq;

  always_comb begin
    // Single interrupt line per UART.
    for (int i = 0; i < UART_NUM; i++) begin
      uart_irq[i] = |uart_interrupts[i];
    end
    // Single interrupt line per I2C device.
    for (int i = 0; i < I2C_NUM; i++) begin
      i2c_irq[i] = |i2c_interrupts[i];
    end
    // Single interrupt line per SPI controller; there are 2 dedicated SPI controllers
    // for the on-board peripherals.
    for (int i = 0; i < TotalSpiNum; i++) begin
      spi_irq[i] = |spi_interrupts[i];
    end
    // Single interrupt line for USBDEV.
    usbdev_irq = |usbdev_interrupts;
    // Single interrupt line for all GPIO Pin-Change Interrupts
    gpio_irq = 1'b0;
    for (int i = 0; i < TotalGpioNum; i++) begin
      gpio_irq |= gpio_interrupts[i];
    end
  end

  logic [31:0] intr_vector;

  assign intr_vector[31               : 24 + TotalSpiNum] = 'b0;      // Support up to 8 SPI controllers.
  assign intr_vector[23 + TotalSpiNum : 24              ] = spi_irq;
  assign intr_vector[23               : 16 + I2C_NUM    ] = 'b0;      // Support up to 8 I2C blocks.
  assign intr_vector[15 + I2C_NUM     : 16              ] = i2c_irq;
  assign intr_vector[15               :  8 + UART_NUM   ] = 'b0;
  assign intr_vector[ 7 + UART_NUM    :  8              ] = uart_irq; // Support up to 8 UARTs.
  assign intr_vector[ 7               :  5              ] = 3'h0;     // Reserved for future use.
  assign intr_vector[ 4                                 ] = gpio_irq;
  assign intr_vector[ 3                                 ] = usbdev_irq;
  assign intr_vector[ 2                                 ] = ethmac_irq;
  assign intr_vector[ 1                                 ] = hardware_revoker_irq;
  assign intr_vector[ 0                                 ] = 1'b0;     // This is a special case and tied to zero.

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
  assign device_req[Timer]    = device_re[Timer]    | device_we[Timer];
  assign device_req[DbgDev]   = device_re[DbgDev]   | device_we[DbgDev];

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

  logic debug_req;

  //////////////////////////////////////////////
  // Instantiate TL-UL crossbar and adapters. //
  //////////////////////////////////////////////

  // Host interfaces.
  tlul_pkg::tl_h2d_t tl_ibex_ins_h2d;
  tlul_pkg::tl_d2h_t tl_ibex_ins_d2h;
  tlul_pkg::tl_h2d_t tl_ibex_lsu_h2d;
  tlul_pkg::tl_d2h_t tl_ibex_lsu_d2h;
  tlul_pkg::tl_h2d_t tl_dbg_host_h2d;
  tlul_pkg::tl_d2h_t tl_dbg_host_d2h;

  // Device interfaces.
  tlul_pkg::tl_h2d_t tl_sram_a_h2d;
  tlul_pkg::tl_d2h_t tl_sram_a_d2h;
  tlul_pkg::tl_h2d_t tl_sram_b_h2d;
  tlul_pkg::tl_d2h_t tl_sram_b_d2h;
  tlul_pkg::tl_h2d_t tl_hyperram_h2d[2];
  tlul_pkg::tl_d2h_t tl_hyperram_d2h[2];
  tlul_pkg::tl_h2d_t tl_gpio_h2d;
  tlul_pkg::tl_d2h_t tl_gpio_d2h;
  tlul_pkg::tl_h2d_t tl_xadc_h2d;
  tlul_pkg::tl_d2h_t tl_xadc_d2h;
  tlul_pkg::tl_h2d_t tl_uart_h2d[UART_NUM];
  tlul_pkg::tl_d2h_t tl_uart_d2h[UART_NUM];
  tlul_pkg::tl_h2d_t tl_timer_h2d;
  tlul_pkg::tl_d2h_t tl_timer_d2h;
  tlul_pkg::tl_h2d_t tl_spi_lcd_h2d;
  tlul_pkg::tl_d2h_t tl_spi_lcd_d2h;
  tlul_pkg::tl_h2d_t tl_spi_ethmac_h2d;
  tlul_pkg::tl_d2h_t tl_spi_ethmac_d2h;
  tlul_pkg::tl_h2d_t tl_system_info_h2d;
  tlul_pkg::tl_d2h_t tl_system_info_d2h;
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
  tlul_pkg::tl_h2d_t tl_dbg_dev_us_h2d[2];
  tlul_pkg::tl_d2h_t tl_dbg_dev_us_d2h[2];
  tlul_pkg::tl_h2d_t tl_usbdev_h2d;
  tlul_pkg::tl_d2h_t tl_usbdev_d2h;
  tlul_pkg::tl_h2d_t tl_rev_tag_h2d;
  tlul_pkg::tl_d2h_t tl_rev_tag_d2h;
  tlul_pkg::tl_h2d_t tl_hw_rev_h2d;
  tlul_pkg::tl_d2h_t tl_hw_rev_d2h;
  tlul_pkg::tl_h2d_t tl_pinmux_h2d;
  tlul_pkg::tl_d2h_t tl_pinmux_d2h;
  tlul_pkg::tl_h2d_t tl_dbg_dev_ds_h2d;
  tlul_pkg::tl_d2h_t tl_dbg_dev_ds_d2h;

  sonata_xbar_main xbar (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),
    .clk_usb_i        (clk_usb_i),
    .rst_usb_ni       (rst_usb_ni),

    // Host interfaces.
    .tl_ibex_lsu_i    (tl_ibex_lsu_h2d),
    .tl_ibex_lsu_o    (tl_ibex_lsu_d2h),
    .tl_dbg_host_i    (tl_dbg_host_h2d),
    .tl_dbg_host_o    (tl_dbg_host_d2h),

    // Device interfaces.
    .tl_sram_o        (tl_sram_a_h2d),
    .tl_sram_i        (tl_sram_a_d2h),
    .tl_hyperram_o    (tl_hyperram_h2d[0]),
    .tl_hyperram_i    (tl_hyperram_d2h[0]),
    .tl_rev_tag_o     (tl_rev_tag_h2d),
    .tl_rev_tag_i     (tl_rev_tag_d2h),
    .tl_gpio_o        (tl_gpio_h2d),
    .tl_gpio_i        (tl_gpio_d2h),
    .tl_pwm_o         ('{tl_pwm_h2d}),
    .tl_pwm_i         ('{tl_pwm_d2h}),
    .tl_system_info_o (tl_system_info_h2d),
    .tl_system_info_i (tl_system_info_d2h),
    .tl_pinmux_o      (tl_pinmux_h2d),
    .tl_pinmux_i      (tl_pinmux_d2h),
    .tl_rgbled_ctrl_o (tl_rgbled_ctrl_h2d),
    .tl_rgbled_ctrl_i (tl_rgbled_ctrl_d2h),
    .tl_hw_rev_o      (tl_hw_rev_h2d),
    .tl_hw_rev_i      (tl_hw_rev_d2h),
    .tl_xadc_o        (tl_xadc_h2d),
    .tl_xadc_i        (tl_xadc_d2h),
    .tl_timer_o       (tl_timer_h2d),
    .tl_timer_i       (tl_timer_d2h),
    .tl_spi_lcd_o     (tl_spi_lcd_h2d),
    .tl_spi_lcd_i     (tl_spi_lcd_d2h),
    .tl_spi_ethmac_o  (tl_spi_ethmac_h2d),
    .tl_spi_ethmac_i  (tl_spi_ethmac_d2h),
    .tl_uart_o        (tl_uart_h2d),
    .tl_uart_i        (tl_uart_d2h),
    .tl_i2c_o         (tl_i2c_h2d),
    .tl_i2c_i         (tl_i2c_d2h),
    .tl_spi_o         (tl_spi_h2d),
    .tl_spi_i         (tl_spi_d2h),
    .tl_usbdev_o      (tl_usbdev_h2d),
    .tl_usbdev_i      (tl_usbdev_d2h),
    .tl_dbg_dev_o     (tl_dbg_dev_us_h2d[1]),
    .tl_dbg_dev_i     (tl_dbg_dev_us_d2h[1]),
    .tl_rv_plic_o     (tl_rv_plic_h2d),
    .tl_rv_plic_i     (tl_rv_plic_d2h)
  );

  xbar_ifetch u_xbar_ifetch (
    // Clock and reset.
    .clk_sys_i        (clk_sys_i),
    .rst_sys_ni       (rst_sys_ni),

    // Host.
    .tl_ibex_ifetch_i (tl_ibex_ins_h2d),
    .tl_ibex_ifetch_o (tl_ibex_ins_d2h),

    // Devices.
    .tl_sram_o     (tl_sram_b_h2d),
    .tl_sram_i     (tl_sram_b_d2h),
    .tl_hyperram_o (tl_hyperram_h2d[1]),
    .tl_hyperram_i (tl_hyperram_d2h[1]),
    .tl_dbg_dev_o  (tl_dbg_dev_us_h2d[0]),
    .tl_dbg_dev_i  (tl_dbg_dev_us_d2h[0]),

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

    .tl_o         (tl_ibex_lsu_h2d),
    .tl_i         (tl_ibex_lsu_d2h)
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

    .tl_o         (tl_dbg_host_h2d),
    .tl_i         (tl_dbg_host_d2h)
  );

  sram #(
    .AddrWidth       ( SRAMAddrWidth   ),
    .DataWidth       ( BusDataWidth    ),
    .DataBitsPerMask ( DataBitsPerMask ),
    .InitFile        ( SRAMInitFile    )
  ) u_sram_top (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .tl_a_i (tl_sram_a_h2d),
    .tl_a_o (tl_sram_a_d2h),
    .tl_b_i (tl_sram_b_h2d),
    .tl_b_o (tl_sram_b_d2h)
  );

`ifdef TARGET_XL_BOARD
  // No HyperRAM on Sonata XL, so the build requires USE_HYPERRAM_SRAM_MODEL
  wire [7:0] hyperram_dq;
  wire       hyperram_rwds;
  wire       hyperram_ckp;
  wire       hyperram_ckn;
  wire       hyperram_nrst;
  wire       hyperram_cs;
  wire       clk_hr_i    = 1'b0;
  wire       clk_hr90p_i = 1'b0;
  wire       clk_hr3x_i  = 1'b0;
  wire       rst_hr_ni   = 1'b0;
`endif

  // HyperRAM
  hyperram #(
    .HyperRAMClkFreq ( HyperRAMClkFreq ),
    .HyperRAMSize    ( HyperRAMSize    ),
    .NumPorts        ( 2               )
  ) u_hyperram (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .clk_hr_i,
    .clk_hr90p_i,
    .clk_hr3x_i,
    .rst_hr_ni,

    .tl_i (tl_hyperram_h2d),
    .tl_o (tl_hyperram_d2h),

    .hyperram_dq,
    .hyperram_rwds,
    .hyperram_ckp,
    .hyperram_ckn,
    .hyperram_nrst,
    .hyperram_cs
  );

  tlul_socket_m1 #(
    .HReqDepth (8'h0),
    .HRspDepth (8'h0),
    .DReqDepth (4'h0),
    .DRspDepth (4'h0),
    .M         (2)
  ) u_debug_module_tl_socket (
    .clk_i (clk_sys_i),
    .rst_ni(rst_sys_ni),
    .tl_h_i(tl_dbg_dev_us_h2d),
    .tl_h_o(tl_dbg_dev_us_d2h),
    .tl_d_o(tl_dbg_dev_ds_h2d),
    .tl_d_i(tl_dbg_dev_ds_d2h)
  );

  tlul_adapter_reg #(
    .RegAw         ( DRegAddrWidth ),
    .AccessLatency ( 1             )
  ) debug_module_device_adapter (
    .clk_i        (clk_sys_i),
    .rst_ni       (rst_sys_ni),

    // TL-UL interface.
    .tl_i         (tl_dbg_dev_ds_h2d),
    .tl_o         (tl_dbg_dev_ds_d2h),

    // Control interface.
    .en_ifetch_i  (prim_mubi_pkg::MuBi4True),
    .intg_error_o (),

    // Register interface.
    .re_o         (device_re[DbgDev]),
    .we_o         (device_we[DbgDev]),
    .addr_o       (device_addr[DbgDev][DRegAddrWidth-1:0]),
    .wdata_o      (device_wdata[DbgDev]),
    .be_o         (device_be[DbgDev]),
    .busy_i       ('0),
    .rdata_i      (device_rdata[DbgDev]),
    .error_i      (device_err[DbgDev])
  );

  assign device_err[DbgDev] = 1'b0;

  // Set upper bits of address.
  assign device_addr[DbgDev][BusAddrWidth-1:DRegAddrWidth] = tl_ifetch_pkg::ADDR_SPACE_DBG_DEV[BusAddrWidth-1:DRegAddrWidth];

  tlul_adapter_reg #(
    .RegAw            ( TRegAddrWidth  ),
    .AccessLatency    ( TAccessLatency )
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

  // Size of revocation tag memory is one bit for each 64 in SRAM.
  localparam int unsigned RevTagDepth = (MemSize / 8) / BusDataWidth;
  localparam int unsigned RevTagAddrWidth = $clog2(RevTagDepth);

  tlul_adapter_sram #(
    .SramAw           ( RevTagAddrWidth )
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
    .rerror_i     (2'b00),

    // Readback functionality not required.
    .compound_txn_in_progress_o (),
    .readback_en_i              (prim_mubi_pkg::MuBi4False),
    .readback_error_o           (),
    .wr_collision_i             (1'b0),
    .write_pending_i            (1'b0)
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
  assign rst_core_n = rst_sys_ni & ~ndmreset_req;

  logic [CheriErrWidth-1:0] cheri_err;

  pwm_fade u_pwm_fade[CheriErrWidth-1:0] (
    .clk_i       (clk_sys_i  ),
    .rst_ni      (rst_core_n ),
    .impulse_i   (cheri_err  ),
    .modulated_o (cheri_err_o)
  );

  ibexc_top_tracing #(
    .DmHaltAddr      ( tl_ifetch_pkg::ADDR_SPACE_DBG_DEV +
                       dm::HaltAddress[31:0]               ),
    .DmExceptionAddr ( tl_ifetch_pkg::ADDR_SPACE_DBG_DEV +
                       dm::ExceptionAddress[31:0]          ),
    .DbgTriggerEn    ( DbgTriggerEn                        ),
    .DbgHwBreakNum   ( DbgHwBreakNum                       ),
    .MHPMCounterNum  ( 13                                  ),
    // For now revocation tags apply to all of SRAM.
    .HeapBase        ( tl_main_pkg::ADDR_SPACE_SRAM        ),
    .TSMapBase       ( tl_main_pkg::ADDR_SPACE_REV_TAG     ),
    .TSMapSize       ( RevTagDepth                         ),
    .RV32M           ( ibex_pkg::RV32MSingleCycle          ),
    .RV32B           ( ibex_pkg::RV32BFull                 ),
    .ICache          ( 1'b1                                )
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

    .debug_req_i            (debug_req),
    .crash_dump_o           (),
    .double_fault_seen_o    (),

    .fetch_enable_i         ('1),
    .alert_minor_o          (  ),
    .alert_major_internal_o (  ),
    .alert_major_bus_o      (  ),
    .core_sleep_o           (  )
  );

  rev_ctl u_rev_ctl (
    .clk_i         (clk_sys_i),
    .rst_ni        (rst_core_n),

    .core_to_ctl_i (hardware_revoker_control_reg_wdata),
    .ctl_to_core_o (hardware_revoker_control_reg_rdata),
    .rev_ctl_irq_o (hardware_revoker_irq),

    .tl_i          (tl_hw_rev_h2d),
    .tl_o          (tl_hw_rev_d2h)
  );

  // GPIOs
  // 0: General Purpose
  // 1: Raspberry Pi HAT
  // 2: Arduino Shield
  // 3: Pmod0
  // 4: Pmod1
  // 5: PmodC
  logic [GPIO_IOS_WIDTH-1:0] gpio_from_pins     [TotalGpioNum];
  logic [GPIO_IOS_WIDTH-1:0] gpio_to_pins       [TotalGpioNum];
  logic [GPIO_IOS_WIDTH-1:0] gpio_to_pins_enable[TotalGpioNum];

  assign gpio_from_pins[0] = gp_i;
  assign gp_o              = gpio_to_pins       [0];
  assign gp_o_en           = gpio_to_pins_enable[0];

  gpio #(
    .GpiMaxWidth  ( GPIO_IOS_WIDTH      ),
    .GpoMaxWidth  ( GPIO_IOS_WIDTH      ),
    .DataWidth    ( BusDataWidth        ),
    .NumInstances ( TotalGpioNum        ),
    .GpiInstWidths( GPIO_INST_IN_WIDTH  ),
    .GpoInstWidths( GPIO_INST_OUT_WIDTH )
  ) u_gpio (
    .clk_i    (clk_sys_i),
    .rst_ni   (rst_sys_ni),

    .tl_i     (tl_gpio_h2d),
    .tl_o     (tl_gpio_d2h),

    .gp_i     (gpio_from_pins),
    .gp_o     (gpio_to_pins),
    .gp_o_en  (gpio_to_pins_enable),

    .pcint_o  (gpio_interrupts)
  );

  // Digital inputs from Arduino shield analog(ue) pins currently unused
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

  // I2C controllers/targets.
  logic i2c_scl_h2d   [I2C_NUM];
  logic i2c_scl_en_h2d[I2C_NUM];
  logic i2c_scl_d2h   [I2C_NUM];
  logic i2c_sda_h2d   [I2C_NUM];
  logic i2c_sda_en_h2d[I2C_NUM];
  logic i2c_sda_d2h   [I2C_NUM];
  for (genvar i = 0; i < I2C_NUM; i++) begin : gen_i2c_hosts
    i2c u_i2c (
      .clk_i                   (clk_sys_i),
      .rst_ni                  (rst_sys_ni),
      .ram_cfg_i               (10'b0),

      // Bus interface.
      .tl_i                    (tl_i2c_h2d[i]),
      .tl_o                    (tl_i2c_d2h[i]),

      // Generic IO.
      .cio_scl_i               (i2c_scl_d2h   [i]),
      .cio_scl_o               (i2c_scl_h2d   [i]),
      .cio_scl_en_o            (i2c_scl_en_h2d[i]),
      .cio_sda_i               (i2c_sda_d2h   [i]),
      .cio_sda_o               (i2c_sda_h2d   [i]),
      .cio_sda_en_o            (i2c_sda_en_h2d[i]),

      // Interrupts.
      .intr_fmt_threshold_o    (i2c_interrupts[i][0]),
      .intr_rx_threshold_o     (i2c_interrupts[i][1]),
      .intr_acq_threshold_o    (i2c_interrupts[i][2]),
      .intr_rx_overflow_o      (i2c_interrupts[i][3]),
      .intr_controller_halt_o  (i2c_interrupts[i][4]),
      .intr_scl_interference_o (i2c_interrupts[i][5]),
      .intr_sda_interference_o (i2c_interrupts[i][6]),
      .intr_stretch_timeout_o  (i2c_interrupts[i][7]),
      .intr_sda_unstable_o     (i2c_interrupts[i][8]),
      .intr_cmd_complete_o     (i2c_interrupts[i][9]),
      .intr_tx_stretch_o       (i2c_interrupts[i][10]),
      .intr_tx_threshold_o     (i2c_interrupts[i][11]),
      .intr_acq_stretch_o      (i2c_interrupts[i][12]),
      .intr_unexp_stop_o       (i2c_interrupts[i][13]),
      .intr_host_timeout_o     (i2c_interrupts[i][14])
    );
  end : gen_i2c_hosts

  // Pulse width modulator.
  logic [PWM_OUT_WIDTH-1:0] pwm_modulated;

  assign lcd_backlight_o = pwm_modulated[PWM_OUT_WIDTH-1];

  pwm_wrapper #(
    .PwmWidth   ( PWM_OUT_WIDTH ),
    .PwmCtrSize ( PwmCtrSize )
  ) u_pwm (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),

    .tl_i   (tl_pwm_h2d),
    .tl_o   (tl_pwm_d2h),

    .pwm_o  (pwm_modulated)
  );

  // UARTs
  //   0 and 1: FTDI chip
  //   2: Raspberry Pi HAT
  //   3: mikroBUS Click
  //   4: RS-232
  logic uart_rx[UART_NUM];
  logic uart_tx[UART_NUM];
  logic uart_tx_en[UART_NUM];
  for (genvar i = 0; i < UART_NUM; i++) begin : gen_uart_blocks
    uart u_uart (
      .clk_i                (clk_sys_i),
      .rst_ni               (rst_sys_ni),

      .cio_rx_i             (uart_rx   [i]),
      .cio_tx_o             (uart_tx   [i]),
      .cio_tx_en_o          (uart_tx_en[i]),

      // Inter-module signals.
      .tl_i                 (tl_uart_h2d[i]),
      .tl_o                 (tl_uart_d2h[i]),

      // Interrupts.
      // Note: the indexes here match the bits in the `intr_` registers,
      // but we also keep the port ordering the same as the module.
      .intr_tx_watermark_o  (uart_interrupts[i][0]),
      .intr_tx_empty_o      (uart_interrupts[i][8]),  // Interrupt was appended.
      .intr_rx_watermark_o  (uart_interrupts[i][1]),
      .intr_tx_done_o       (uart_interrupts[i][2]),
      .intr_rx_overflow_o   (uart_interrupts[i][3]),
      .intr_rx_frame_err_o  (uart_interrupts[i][4]),
      .intr_rx_break_err_o  (uart_interrupts[i][5]),
      .intr_rx_timeout_o    (uart_interrupts[i][6]),
      .intr_rx_parity_err_o (uart_interrupts[i][7])
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
    .intr_pkt_received_o          (usbdev_interrupts[0]),
    .intr_pkt_sent_o              (usbdev_interrupts[1]),
    .intr_powered_o               (usbdev_interrupts[2]),
    .intr_disconnected_o          (usbdev_interrupts[3]),
    .intr_host_lost_o             (usbdev_interrupts[4]),
    .intr_link_reset_o            (usbdev_interrupts[5]),
    .intr_link_suspend_o          (usbdev_interrupts[6]),
    .intr_link_resume_o           (usbdev_interrupts[7]),
    .intr_av_out_empty_o          (usbdev_interrupts[8]),
    .intr_rx_full_o               (usbdev_interrupts[9]),
    .intr_av_overflow_o           (usbdev_interrupts[10]),
    .intr_link_in_err_o           (usbdev_interrupts[11]),
    .intr_link_out_err_o          (usbdev_interrupts[12]),
    .intr_rx_crc_err_o            (usbdev_interrupts[13]),
    .intr_rx_pid_err_o            (usbdev_interrupts[14]),
    .intr_rx_bitstuff_err_o       (usbdev_interrupts[15]),
    .intr_frame_o                 (usbdev_interrupts[16]),
    .intr_av_setup_empty_o        (usbdev_interrupts[17])
  );

  // Dedicated Spi Controllers
  // - LCD screen
  // - Ethernet MAC
  spi #(
    .CSWidth(4)
  ) u_spi_lcd (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    // TileLink interface.
    .tl_i                (tl_spi_lcd_h2d),
    .tl_o                (tl_spi_lcd_d2h),

    // Interrupts currently disconnected.
    .intr_rx_full_o      (spi_interrupts[0][0]),
    .intr_rx_watermark_o (spi_interrupts[0][1]),
    .intr_tx_empty_o     (spi_interrupts[0][2]),
    .intr_tx_watermark_o (spi_interrupts[0][3]),
    .intr_complete_o     (spi_interrupts[0][4]),

    // SPI signals.
    .spi_copi_o          (lcd_copi_o),
    .spi_cipo_i          (lcd_cipo_i),
    .spi_cs_o            ({lcd_copi_en_o, lcd_rst_o, lcd_dc_o, lcd_cs_o}),
    .spi_clk_o           (lcd_sclk_o)
  );

  spi #(
    .CSWidth(2)
  ) u_spi_ethmac (
    .clk_i               (clk_sys_i),
    .rst_ni              (rst_sys_ni),

    // TileLink interface.
    .tl_i                (tl_spi_ethmac_h2d),
    .tl_o                (tl_spi_ethmac_d2h),

    // Interrupts currently disconnected.
    .intr_rx_full_o      (spi_interrupts[1][0]),
    .intr_rx_watermark_o (spi_interrupts[1][1]),
    .intr_tx_empty_o     (spi_interrupts[1][2]),
    .intr_tx_watermark_o (spi_interrupts[1][3]),
    .intr_complete_o     (spi_interrupts[1][4]),

    // SPI signals.
    .spi_copi_o          (ethmac_copi_o),
    .spi_cipo_i          (ethmac_cipo_i),
    .spi_cs_o            ({ethmac_rst_o, ethmac_cs_o}),
    .spi_clk_o           (ethmac_sclk_o)
  );

  // Pinmuxed SPI controllers.
  // - 2x Pinmuxed
  logic                    spi_sclk[SPI_NUM];
  logic                    spi_copi[SPI_NUM];
  logic                    spi_cipo[SPI_NUM];
  logic [SPI_CS_WIDTH-1:0] spi_cs[SPI_NUM];

  for (genvar i = 0; i < SPI_NUM; i++) begin : gen_spi_hosts
    spi #(
      .CSWidth(SPI_CS_WIDTH)
    ) u_spi (
      .clk_i               (clk_sys_i),
      .rst_ni              (rst_sys_ni),

      // TileLink interface.
      .tl_i                (tl_spi_h2d[i]),
      .tl_o                (tl_spi_d2h[i]),

      // Interrupts currently disconnected.
      .intr_rx_full_o      (spi_interrupts[i + FixedSpiNum][0]),
      .intr_rx_watermark_o (spi_interrupts[i + FixedSpiNum][1]),
      .intr_tx_empty_o     (spi_interrupts[i + FixedSpiNum][2]),
      .intr_tx_watermark_o (spi_interrupts[i + FixedSpiNum][3]),
      .intr_complete_o     (spi_interrupts[i + FixedSpiNum][4]),

      // SPI signals.
      .spi_copi_o          (spi_copi[i]),
      .spi_cipo_i          (spi_cipo[i]),
      .spi_cs_o            (spi_cs[i]),
      .spi_clk_o           (spi_sclk[i])
    );
  end : gen_spi_hosts

  // Sample the ethernet interrupt pin.
  always_ff @(posedge clk_sys_i or negedge rst_sys_ni) begin
    if (!rst_sys_ni) begin
      ethmac_irq <= 1'b0;
    end else begin
      ethmac_irq <= !ethmac_irq_ni;
    end
  end

  // RISC-V timer.
  rv_timer #(
    .DataWidth    ( BusDataWidth   ),
    .AddressWidth ( BusAddrWidth   ),
    .AccessLatency( TAccessLatency )
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

    .intr_src_i (intr_vector),

    .msip_o     ()
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
    .debug_req_o    (debug_req),
    .unavailable_i  (1'b0),

    .cheri_en_i     (cheri_en),  // CHERIoT enabled?

    // Bus device with debug memory (for execution-based debug).
    .device_req_i   (device_req[DbgDev]),
    .device_we_i    (device_we[DbgDev]),
    .device_addr_i  (device_addr[DbgDev]),
    .device_be_i    (device_be[DbgDev]),
    .device_wdata_i (device_wdata[DbgDev]),
    .device_rdata_o (device_rdata[DbgDev]),

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

  // Debug module is not capability-aware.
  assign host_wcap[DbgHost] = 1'b0;

  system_info #(
    .SysClkFreq (   SysClkFreq ),
    .GpioNum    ( TotalGpioNum ),
    .UartNum    (     UART_NUM ),
    .I2cNum     (      I2C_NUM ),
    .SpiNum     (  TotalSpiNum )
  ) u_system_info (
    .clk_i  (clk_sys_i),
    .rst_ni (rst_sys_ni),
    .tl_i   (tl_system_info_h2d),
    .tl_o   (tl_system_info_d2h)
  );

  // Output Pins
  // Pull output pins high when their output isn't enabled.
  // They are pulled high because SPI CS and UART TX lines
  // should default to being pulled high.
  sonata_out_pins_t out_to_pins_data, out_to_pins_en;
  assign out_to_pins_o = out_to_pins_data | ~out_to_pins_en;

  pinmux u_pinmux (
    .clk_i(clk_sys_i),
    .rst_ni(rst_sys_ni),

    .pwm_out_i('{PWM_NUM{pwm_modulated}}),
    .pwm_out_en_i('{PWM_NUM{{PWM_OUT_WIDTH{1'b1}}}}),

    .uart_rx_o(uart_rx),
    .uart_tx_i(uart_tx),
    .uart_tx_en_i(uart_tx_en),

    .i2c_scl_o(i2c_scl_d2h),
    .i2c_scl_i(i2c_scl_h2d),
    .i2c_scl_en_i(i2c_scl_en_h2d),
    .i2c_sda_o(i2c_sda_d2h),
    .i2c_sda_i(i2c_sda_h2d),
    .i2c_sda_en_i(i2c_sda_en_h2d),

    .spi_cipo_o(spi_cipo),
    .spi_copi_i(spi_copi),
    .spi_copi_en_i('{default: 'b1}),
    .spi_sclk_i(spi_sclk),
    .spi_sclk_en_i('{default: 'b1}),
    .spi_cs_i(spi_cs),
    .spi_cs_en_i('{default: '1}), // All continuously enabled.

    .gpio_ios_o(gpio_from_pins[1:GPIO_NUM]),
    .gpio_ios_i(gpio_to_pins[1:GPIO_NUM]),
    .gpio_ios_en_i(gpio_to_pins_enable[1:GPIO_NUM]),

    .in_from_pins_i,
    .out_to_pins_o(out_to_pins_data),
    .out_to_pins_en_o(out_to_pins_en),
    .inout_from_pins_i,
    .inout_to_pins_o,
    .inout_to_pins_en_o,

    .tl_i(tl_pinmux_h2d),
    .tl_o(tl_pinmux_d2h)
  );

  // The RS-485 transceiver operates in half-duplex, it cannot transmit and receive at the same time.
  // When transmitting the driver must be enabled and otherwise it must be disabled to receive. This
  // is handled by the rs485_ctrl module in the hierarchy above sonata_system but that module
  // requires rx_enable and tx_enable inputs which are provided below.

  // Enable the RS-485 receiver any time the RS-485 RX input is connect to the UART via pinmux.
  // Whether or not we are transmitting does not need to be factored in here as that is dealt with
  // via rs485_ctrl.
  assign rs485_rx_enable_o = u_pinmux.uart_rx_2_sel[3];

  // Transmission enabled when UART is muxed to RS-485 TX output and UART is actively transmitting.
  assign rs485_tx_enable_o = u_pinmux.rs485_tx_sel[1]                           &
                             gen_uart_blocks[2].u_uart.uart_core.tx_enable      &
                             ~gen_uart_blocks[2].u_uart.hw2reg.status.txidle.d;

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

`ifdef TARGET_XL_BOARD
  logic unused_hr;
  assign unused_hr = ^{hyperram_ckp, hyperram_ckn, hyperram_cs, hyperram_nrst};
`endif
endmodule
