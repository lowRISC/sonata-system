// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level that connects the system to the virtual devices.
module top_verilator (input logic clk_i, rst_ni);
  import sonata_pkg::*;

  // System clock frequency.
  localparam int unsigned SysClkFreq = 40_000_000;
  // HyperRAM clock frequency.
  localparam int unsigned HyperRAMClkFreq  = 100_000_000;
  localparam int unsigned BaudRate   = 921_600;
  // Number of CHERI error LEDs.
  localparam int unsigned CheriErrWidth = 9;
  // The symbolic file descriptors are presently unknown to Verilator
  // (described in IEEE 1800-2012).
  localparam int unsigned STDERR = 32'h8000_0002;

  logic uart_sys_rx, uart_sys_tx;
  logic uart_aux_rx, uart_aux_tx;
  assign uart_aux_rx = 1'b1;

  logic scl_rpi0_o, scl_rpi0_oe;
  logic sda_rpi0_o, sda_rpi0_oe;

  logic scl_rpi1_o, scl_rpi1_oe;
  logic sda_rpi1_o, sda_rpi1_oe;

  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  logic cs_pmod1_o, cs_pmod1_oe;
  logic sck_pmod1_o, sck_pmod1_oe;
  logic copi_pmod1_o, copi_pmod1_oe;

  // Output clocks and data to the I2C buses.
  wire scl_rpi0_out = scl_rpi0_oe ? scl_rpi0_o : 1'b1;
  wire sda_rpi0_out = sda_rpi0_oe ? sda_rpi0_o : 1'b1;

  wire scl_rpi1_out = scl_rpi1_oe ? scl_rpi1_o : 1'b1;
  wire sda_rpi1_out = sda_rpi1_oe ? sda_rpi1_o : 1'b1;

  wire scl1_out = scl1_oe ? scl1_o : 1'b1;
  wire sda1_out = sda1_oe ? sda1_o : 1'b1;

  // Output for the SPI PMOD1 buses, used for PMOD SF3 DPI
  wire cs_pmod1_out   = cs_pmod1_oe ? cs_pmod1_o : 1'b1;
  wire sck_pmod1_out  = sck_pmod1_oe ? sck_pmod1_o : 1'b1;
  wire copi_pmod1_out = copi_pmod1_oe ? copi_pmod1_o : 1'b1;

  // Input from SPI PMOD1 bus
  wire cipo_pmod1_i;

  // SPI PMOD SF3 OOB
  wire pmod1_spi_d2; // WP_N (write protect negated)
  wire pmod1_spi_d3; // HOLD_N or RESET_N
  // Tie to 1 as they're active low and we don't need either signal
  assign pmod1_spi_d2 = 1'b1;
  assign pmod1_spi_d3 = 1'b1;

  // Clocks and data from the I2C DPI models.
  wire scl_rpi0_dpi, sda_rpi0_dpi;
  wire scl_rpi1_dpi, sda_rpi1_dpi;
  wire scl1_dpi, sda1_dpi;

  // Input clocks and data from the I2C buses; these signals must reflect the physical I2C bus,
  // ie. they carry both the outbound and the inbound activity, because otherwise the controller
  // will perceive a mismatch between its own transmissions and the inputs as bus contention.
  wire scl_rpi0_in = scl_rpi0_out & scl_rpi0_dpi;
  wire sda_rpi0_in = sda_rpi0_out & sda_rpi0_dpi;

  wire scl_rpi1_in = scl_rpi1_out & scl_rpi1_dpi;
  wire sda_rpi1_in = sda_rpi1_out & sda_rpi1_dpi;

  wire scl1_in = scl1_out & scl1_dpi;
  wire sda1_in = sda1_out & sda1_dpi;

  wire unused_ = uart_aux_tx;

  // Simplified clocking scheme for simulations.
  wire clk_usb   = clk_i;
  wire rst_usb_n = rst_ni;
  wire rst_hr_n  = rst_ni;

  // In Verilator simulation where tri-stated drivers, pullups/pulldowns and drive strengths are
  // not available, the USBDPI model is connected directly to the two-state inputs and outputs of
  // USBDEV itself, i.e. these are not actual USB signals but rather two separated unidirectional
  // buses.
  //
  // USB signals into the USB device from the DPI/host model; these model the VBUS/SENSE and data
  // signals from the on-board TUSB1106 USB transceiver.
  wire usb_sense;
  wire usb_dp_p2d; // D+, differential signaling.
  wire usb_dn_p2d; // D-
  wire usb_d_p2d;  // D, simulated output from differential receiver.

  // USB signals into the DPI/host model from the USB device; these model the signals from the
  // Sonata FPGA to the on-board TUSB1106 transceiver, as well as permitting the differential
  // receiver enable/disable to be tested.
  wire usb_dp_d2p; // D+, differential signaling.
  wire usb_dn_d2p; // D-
  wire usb_dp_en_d2p; // D+ driver enable.
  wire usb_dn_en_d2p; // D- driver enable.
  wire usb_rx_enable; // Enable differential receiver.
  wire usb_dp_pullup; // D+ pullup enable.
  wire usb_dn_pullup; // D- pullup enable.

  // SPI flash interface.
  wire appspi_clk = out_to_pins[OUT_PIN_APPSPI_CLK];
  // COPI (controller output peripheral input)
  wire appspi_d0 = out_to_pins[OUT_PIN_APPSPI_D0];
  // CIPO (controller input peripheral output)
  wire appspi_d1;
  assign in_from_pins[IN_PIN_APPSPI_D1] = appspi_d1;
  // WP_N (write protect negated)
  wire appspi_d2 = 1'b1;
  // HOLD_N or RESET_N
  wire appspi_d3 = 1'b1;
  // Chip select negated
  wire appspi_cs = out_to_pins[OUT_PIN_APPSPI_CS];

  // microSD card interface.
  wire microsd_clk;  // SPI mode: SCLK
  wire microsd_dat0; // SPI mode: CIPO
  wire microsd_dat3; // SPI mode: CS_N
  wire microsd_cmd;  // SPI mode: COPI
  wire microsd_det;  // microSD card detection; 0 = card present.

  // LCD interface.
  wire lcd_rst;
  wire lcd_dc;
  // SPI-ish interface to LCD.
  wire lcd_copi;
  wire lcd_copi_en;
  wire lcd_copi_out = lcd_copi_en ? lcd_copi : lcd_cipo_in;
  wire lcd_cipo_in;
  wire lcd_cipo = lcd_copi_en ? lcd_copi : lcd_cipo_in;
  wire lcd_clk;
  wire lcd_cs;
  // LCD backlight on/off.
  wire lcd_backlight;

  // mikroBUS Click.
  wire mb1;
  // Arduino
  wire ah_tmpio10;
  // RPi header.
  wire rph_g18, rph_g17, rph_g16_ce2, rph_g8_ce0, rph_g7_ce1;
  // User LEDs.
  wire [7:0] usrLed;
  // None of these signals is used presently.
  wire unused_io_ = ^{mb1, ah_tmpio10, rph_g18, rph_g17,
                      rph_g16_ce2, rph_g8_ce0, rph_g7_ce1,
                      usrLed};

  // Reporting of CHERI enable/disable and any exceptions that occur.
  wire  [CheriErrWidth-1:0] cheri_err;
  logic [CheriErrWidth-1:0] cheri_errored;
  logic cheri_en;

  initial begin : cheri_en_set_and_report
    if ($test$plusargs("disable_cheri")) begin
      cheri_en = 1'b0;
      $display("Running in legacy software mode");
    end else begin
      cheri_en = 1'b1;
      $display("Running with CHERI enabled");
    end
  end : cheri_en_set_and_report

  always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cheri_errored <= '0;
    else if (|(cheri_err & ~cheri_errored)) begin : cheri_err_reporting
      // Report the first occurrence of each exception by name.
      for (int unsigned e = 0; e < CheriErrWidth; e++) begin
        if (cheri_err[e] & !cheri_errored[e]) begin
          string name;
          case (e)
            0: name = "Bounds";
            1: name = "Tag";
            2: name = "Seal";
            3: name = "Permit Execute";
            4: name = "Permit Load";
            5: name = "Permit Store";
            6: name = "Permit Store Cap";
            7: name = "Permit Store Local Cap";
            8: name = "Permit Acc Sys Regs";
            default: name = "Unknown";
          endcase
          // Ensure that the output is visible promptly.
          $fdisplay(STDERR, "*** CHERI '%s' violation occurred *** at time %t", name, $time);
          $fflush(STDERR);
          // Remember that this error occurred; each error signal will be asserted many times
          // because they are intended to drive LEDs on the FPGA board and are thus modulated.
          cheri_errored <= cheri_errored | cheri_err;
        end
      end
    end
  end

  sonata_in_pins_t in_from_pins;
  sonata_out_pins_t out_to_pins;
  sonata_inout_pins_t inout_from_pins, inout_to_pins, inout_to_pins_en;

  logic rs485_tx, rs485_rx;
  logic rs485_tx_enable, rs485_rx_enable;

  assign uart_sys_tx = out_to_pins[OUT_PIN_SER0_TX];
  assign uart_aux_tx = out_to_pins[OUT_PIN_SER1_TX];
  assign rs485_tx    = out_to_pins[OUT_PIN_RS485_TX];

  // Traffic to/from microSD card.
  assign microsd_cmd  = out_to_pins[OUT_PIN_MICROSD_CMD ];
  assign microsd_clk  = out_to_pins[OUT_PIN_MICROSD_CLK ];
  assign microsd_dat3 = out_to_pins[OUT_PIN_MICROSD_DAT3];
  assign in_from_pins[IN_PIN_MICROSD_DAT0] = microsd_dat0;

  // Output I2C traffic to the RPi HAT ID EEPROM.
  assign {scl_rpi0_o, scl_rpi0_oe} = {inout_to_pins[INOUT_PIN_RPH_G1],
                                   inout_to_pins_en[INOUT_PIN_RPH_G1]};
  assign {sda_rpi0_o, sda_rpi0_oe} = {inout_to_pins[INOUT_PIN_RPH_G0],
                                   inout_to_pins_en[INOUT_PIN_RPH_G0]};
  // Output I2C traffic to the secondary I2C bus on the Raspberry Pi HAT (shared with GPIO2/3).
  assign {scl_rpi1_o, scl_rpi1_oe} = {inout_to_pins[INOUT_PIN_RPH_G3_SCL],
                                   inout_to_pins_en[INOUT_PIN_RPH_G3_SCL]};
  assign {sda_rpi1_o, sda_rpi1_oe} = {inout_to_pins[INOUT_PIN_RPH_G2_SDA],
                                   inout_to_pins_en[INOUT_PIN_RPH_G2_SDA]};

  assign {scl1_o, scl1_oe} = {inout_to_pins[INOUT_PIN_SCL1], inout_to_pins_en[INOUT_PIN_SCL1]};
  assign {sda1_o, sda1_oe} = {inout_to_pins[INOUT_PIN_SDA1], inout_to_pins_en[INOUT_PIN_SDA1]};

  // Output SPI traffic to the PMOD SF3 on PMOD1
  assign {cs_pmod1_o, cs_pmod1_oe} = {inout_to_pins[INOUT_PIN_PMOD1_1],
                                   inout_to_pins_en[INOUT_PIN_PMOD1_1]};
  assign {copi_pmod1_o, copi_pmod1_oe} = {inout_to_pins[INOUT_PIN_PMOD1_2],
                                       inout_to_pins_en[INOUT_PIN_PMOD1_2]};
  assign {sck_pmod1_o, sck_pmod1_oe} = {inout_to_pins[INOUT_PIN_PMOD1_4],
                                     inout_to_pins_en[INOUT_PIN_PMOD1_4]};

  assign in_from_pins[IN_PIN_SER0_RX]   = uart_sys_rx;
  assign in_from_pins[IN_PIN_SER1_RX]   = uart_aux_rx;
  assign in_from_pins[IN_PIN_RS485_RX]  = rs485_rx;

  // SCL0/SDA0 pins are presently not connected to any I2C models; just pulled up on the PCB.
  // - there is no model on either the QWIIC0 connector or the Arduino Shield.
  assign inout_from_pins[INOUT_PIN_SCL0] = 1'b1;
  assign inout_from_pins[INOUT_PIN_SDA0] = 1'b1;
  // SCL1/SDA1 has a device model on the QWIIC1 connector.
  assign inout_from_pins[INOUT_PIN_SCL1] = scl1_in;
  assign inout_from_pins[INOUT_PIN_SDA1] = sda1_in;
  // RPi HAT ID bus has a device model.
  assign inout_from_pins[INOUT_PIN_RPH_G0] = sda_rpi0_in;
  assign inout_from_pins[INOUT_PIN_RPH_G1] = scl_rpi0_in;
  // RPi HAT secondary I2C bus also has a device model (Sense HAT).
  assign inout_from_pins[INOUT_PIN_RPH_G2_SDA] = sda_rpi1_in;
  assign inout_from_pins[INOUT_PIN_RPH_G3_SCL] = scl_rpi1_in;
  // There is no device model on the mikroBUS Click I2C bus.
  assign inout_from_pins[INOUT_PIN_MB5] = 1'b1;
  assign inout_from_pins[INOUT_PIN_MB6] = 1'b1;
  // Input SPI traffic from PMOD SF3 on PMOD1
  assign inout_from_pins[INOUT_PIN_PMOD1_3] = cipo_pmod1_i;


  // CS outputs to SPI peripherals from controllers.
  assign rph_g8_ce0   = inout_to_pins[INOUT_PIN_RPH_G8];
  assign rph_g7_ce1   = inout_to_pins[INOUT_PIN_RPH_G7];
  assign ah_tmpio10   = inout_to_pins[INOUT_PIN_AH_TMPIO10];
  assign rph_g18      = inout_to_pins[INOUT_PIN_RPH_G18];
  assign rph_g17      = inout_to_pins[INOUT_PIN_RPH_G17];
  assign rph_g16_ce2  = inout_to_pins[INOUT_PIN_RPH_G16];
  assign mb1          = out_to_pins[OUT_PIN_MB1];

  logic unused_out_pins = ^{out_to_pins[OUT_PIN_RS232_TX],
                            out_to_pins[OUT_PIN_MB2]};
  logic [23:0] unused_gp_o;

  // Loopback functionality used to verify the operation of the pinmux and GPIO pins;
  // these signals are re-timed through a single register stage simply to prevent Verilator
  // warnings about circular combinational logic which assesses circularity at the net level
  // (i.e. `in_from_pins` and `out_to_pins`) rather than the bit level.
  reg [5:0] loopback_q;
  always @(posedge clk_i) begin
    loopback_q <= {inout_to_pins[INOUT_PIN_AH_TMPIO8],
                   inout_to_pins[INOUT_PIN_AH_TMPIO1],
                   out_to_pins[OUT_PIN_MB10],  // mikroBUS CLick PWM -> PMOD0.1; PWM loopback
                   out_to_pins[OUT_PIN_MB7],   // mikroBUS Click TX -> RX; UART loopback.
                   out_to_pins[OUT_PIN_MB4],  // mikroBUS Click COPI -> CIPO; SPI loopback.
                   inout_to_pins[INOUT_PIN_PMOD0_8]}; // PMOD0 8->10; PWM loopback
  end
  assign {inout_from_pins[INOUT_PIN_AH_TMPIO9],
          inout_from_pins[INOUT_PIN_AH_TMPIO0],
          inout_from_pins[INOUT_PIN_PMOD0_1],
          in_from_pins[IN_PIN_MB8],
          in_from_pins[IN_PIN_MB3],
          inout_from_pins[INOUT_PIN_PMOD0_10]} = loopback_q;

  // JTAG signals
  wire jtag_tck;
  wire jtag_tms;
  wire jtag_tdi;
  wire jtag_tdo;
  wire jtag_trst;

  // Switch inputs have pull-ups and switches pull to ground when on, but in `top_sonata`
  // they are inverted, so 0 here means 'not pressed' or 'off'.
  wire [4:0] nav_sw_n = '0;
  wire [7:0] user_sw_n = '0;
  wire [2:0] sel_sw_n = '0;

  // Instantiating the Sonata System.
  sonata_system #(
    .CheriErrWidth   ( CheriErrWidth   ),
    .SysClkFreq      ( SysClkFreq      ),
    .HyperRAMClkFreq ( HyperRAMClkFreq )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_i),
    .rst_sys_ni     (rst_ni),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    // SRAM model used for hyperram so no hyperram clock is provided
    .clk_hr_i       (1'b0),
    .clk_hr90p_i    (1'b0),
    .clk_hr3x_i     (1'b0),
    .rst_hr_ni      (rst_hr_n),

    .gp_i           ({
                      15'b0,
                      microsd_det, // MicroSD card insertion detection
                      sel_sw_n, // Software selection switches
                      nav_sw_n, // joystick
                      user_sw_n // user switches
                    }),
    .gp_o           ({
                      unused_gp_o,
                      usrLed // User LEDs (8 bits)
                     }),

    .gp_o_en      ( ),

    // Arduino Shield Analog(ue)
    .ard_an_di_i    (0),
    .ard_an_p_i     (0),
    .ard_an_n_i     (0),


    // Non-pinmuxed spi devices
    .lcd_copi_o              (lcd_copi),
    .lcd_copi_en_o           (lcd_copi_en),
    .lcd_cipo_i              (lcd_cipo),
    .lcd_sclk_o              (lcd_clk),
    .lcd_cs_o                (lcd_cs),
    .lcd_dc_o                (lcd_dc),
    .lcd_rst_o               (lcd_rst),
    .lcd_backlight_o         (lcd_backlight),

    .ethmac_copi_o           (),
    .ethmac_cipo_i           (),
    .ethmac_sclk_o           (),
    .ethmac_cs_o             (),
    .ethmac_rst_o            (),
    .ethmac_irq_ni           (1'b1), // Interrupt for Ethernet is out of band

    // CHERI signals
    .cheri_en_i     (cheri_en ),
    .cheri_err_o    (cheri_err),
    .cheri_en_o     (         ),

    // Reception from USB host via transceiver
    .usb_dp_i         (usb_dp_p2d),
    .usb_dn_i         (usb_dn_p2d),
    .usb_rx_d_i       (usb_d_p2d),

    // Transmission to USB host via transceiver
    .usb_dp_o         (usb_dp_d2p),
    .usb_dp_en_o      (usb_dp_en_d2p),
    .usb_dn_o         (usb_dn_d2p),
    .usb_dn_en_o      (usb_dn_en_d2p),

    // Configuration and control of USB transceiver
    .usb_sense_i      (usb_sense),
    .usb_dp_pullup_o  (usb_dp_pullup),
    .usb_dn_pullup_o  (usb_dn_pullup),
    .usb_rx_enable_o  (usb_rx_enable),

    // User JTAG
    .tck_i  (jtag_tck),
    .tms_i  (jtag_tms),
    .trst_ni(jtag_trst),
    .td_i   (jtag_tdi),
    .td_o   (jtag_tdo),

    .rgbled_dout_o (),

    // SRAM model used for hyperram so don't connect hyperram IO
    .hyperram_dq  (),
    .hyperram_rwds(),
    .hyperram_ckp (),
    .hyperram_ckn (),
    .hyperram_nrst(),
    .hyperram_cs  (),

    .rs485_tx_enable_o(rs485_tx_enable),
    .rs485_rx_enable_o(rs485_rx_enable),

    .in_from_pins_i     (in_from_pins    ),
    .out_to_pins_o      (out_to_pins     ),
    .inout_from_pins_i  (inout_from_pins ),
    .inout_to_pins_o    (inout_to_pins   ),
    .inout_to_pins_en_o (inout_to_pins_en)
  );

  // I2C HAT ID DPI - this I2C bus is to the ID EEPROM of a Raspberry Pi HAT.
  i2cdpi #(
    .ID   ("i2c_rpi0")
  ) u_i2c_rpi0_dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl_rpi0_out),
    .sda_i    (sda_rpi0_out),
    .scl_o    (scl_rpi0_dpi),
    .sda_o    (sda_rpi0_dpi),
    // Out-Of-Band data.
    .oob_in   (1'b0),
    .oob_out  ()  // not used
  );

  // I2C GPIO2/3 - this I2c bus is also present on the Raspberry Pi HATs,
  // and is used on the Raspberry Pi Sense HAT, for example.
  i2cdpi #(
    .ID   ("i2c_rpi1")
  ) u_i2c_rpi1_dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl_rpi1_out),
    .sda_i    (sda_rpi1_out),
    .scl_o    (scl_rpi1_dpi),
    .sda_o    (sda_rpi1_dpi),
    // Out-Of-Band data.
    .oob_in   (1'b0),
    .oob_out  ()  // not used
  );

  // I2C QWIIC1
  i2cdpi #(
    .ID   ("i2c1")
  ) u_i2c1_dpi (
    .rst_ni   (rst_ni),
    // The connected signal names are from the perspective of the controller.
    .scl_i    (scl1_out),
    .sda_i    (sda1_out),
    .scl_o    (scl1_dpi),
    .sda_o    (sda1_dpi),
    // Out-Of-Band data.
    .oob_in   (1'b0),
    .oob_out  ()  // not used
  );

  // Virtual JTAG
  jtagdpi u_jtagdpi (
    .clk_i,
    .rst_ni,

    .jtag_tck    (jtag_tck),
    .jtag_tms    (jtag_tms),
    .jtag_tdi    (jtag_tdi),
    .jtag_tdo    (jtag_tdo),
    .jtag_trst_n (jtag_trst),
    .jtag_srst_n ( )
  );

  // Virtual UART
  uartdpi #(
    .BAUD ( BaudRate    ),
    .FREQ ( SysClkFreq  )
  ) u_uartdpi (
    .clk_i,
    .rst_ni,
    .active(1'b1       ),
    .tx_o  (uart_sys_rx),
    .rx_i  (uart_sys_tx)
  );

  // USB DPI; simulated USB host.
  usbdpi u_usbdpi (
    .clk_i           (clk_usb),
    .rst_ni          (rst_usb_n),
    .clk_48MHz_i     (clk_usb),
    .enable          (1'b1),
    // D+ drivers and their enables.
    .dp_en_p2d       (),
    .dp_p2d          (usb_dp_p2d),
    .dp_d2p          (usb_dp_d2p),
    .dp_en_d2p       (usb_dp_en_d2p),
    // D- drivers and their enables.
    .dn_en_p2d       (),
    .dn_p2d          (usb_dn_p2d),
    .dn_d2p          (usb_dn_d2p),
    .dn_en_d2p       (usb_dn_en_d2p),
    // D drivers (used when external differential receiver is enabled).
    .d_p2d           (usb_d_p2d),
    .d_d2p           (1'b0),
    .d_en_d2p        (1'b0),
    .se0_d2p         (1'b0),
    // Enable signal for external differential receiver.
    .rx_enable_d2p   (usb_rx_enable),
    // Sonata FPGA does not employ D/SE0 signaling in place of D+/D-.
    .tx_use_d_se0_d2p(1'b0),

    // VBUS/SENSE signal indicating the presence of the USB host.
    .sense_p2d       (usb_sense),
    // Pullup enables from the USB device.
    .pullupdp_d2p    (usb_dp_pullup),
    .pullupdn_d2p    (usb_dn_pullup)
  );

  // SPI connection to flash.
  spidpi #(
    .ID       ("flash"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (2),
    .OOB_OutW (1)
  ) u_spidpi_flash (
    .rst_ni   (rst_ni),

    .sck      (appspi_clk),
    .cs       (appspi_cs),
    .copi     (appspi_d0),
    .cipo     (appspi_d1),

    .oob_in   ({appspi_d3, appspi_d2}),
    .oob_out  ( )
  );

  // SPI connection to LCD.
  spidpi #(
    .ID       ("lcd"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (3),
    .OOB_OutW (1)
  ) u_spidpi_lcd (
    .rst_ni   (rst_ni),

    .sck      (lcd_clk),
    .cs       (lcd_cs),
    .copi     (lcd_copi_out),
    .cipo     (lcd_cipo_in),

    .oob_in   ({lcd_dc, lcd_rst, lcd_backlight}),
    .oob_out  ( )  // not used.
  );

  // SPI connection to microSD card.
  spidpi #(
    .ID       ("microsd"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (1),
    .OOB_OutW (1)
  ) u_spidpi_microsd (
    .rst_ni   (rst_ni),

    .sck      (microsd_clk),
    .cs       (microsd_dat3),
    .copi     (microsd_cmd),
    .cipo     (microsd_dat0),

    .oob_in   ( ),
    .oob_out  (microsd_det)
  );

  // SPI connection to PMOD SF3 flash via PMOD1 pins
  spidpi #(
    .ID       ("pmod_sf3"),
    .NDevices (1),
    .DataW    (1),
    .OOB_InW  (2),
    .OOB_OutW (1)
  ) u_spidpi_pmod_sf3 (
    .rst_ni   (rst_ni),

    .sck      (sck_pmod1_out),
    .cs       (cs_pmod1_out),
    .copi     (copi_pmod1_out),
    .cipo     (cipo_pmod1_i),

    .oob_in   ({pmod1_spi_d3, pmod1_spi_d2}),
    .oob_out  ( )  // not used.
  );

  logic rs485_di, rs485_ro;
  logic rs485_ren, rs485_de;

  rs485_ctrl u_rs485_ctrl (
    .clk_i (clk_i),
    .rst_ni(rst_ni),

    .tx_i       (rs485_tx),
    .rx_o       (rs485_rx),
    .rx_enable_i(rs485_rx_enable),
    .tx_enable_i(rs485_tx_enable),

    .di_o (rs485_di),
    .ren_o(rs485_ren),
    .de_o (rs485_de),
    .ro_i (rs485_ro)
  );

  logic rs485_uartdpi_tx, rs485_uartdpi_rx;

  assign rs485_ro =         rs485_ren ? 1'b0     : rs485_uartdpi_tx;
  assign rs485_uartdpi_rx = rs485_de  ? rs485_di : 1'b1;

  uartdpi #(
    .BAUD ( BaudRate    ),
    .FREQ ( SysClkFreq  ),
    .NAME ( "rs485"     )
  ) u_rs485_uartdpi (
    .clk_i,
    .rst_ni,
    .active(1'b1       ),
    .tx_o  (rs485_uartdpi_tx),
    .rx_i  (rs485_uartdpi_rx)
  );

  export "DPI-C" function mhpmcounter_get;

  function automatic longint unsigned mhpmcounter_get(int index);
    return u_sonata_system.u_top_tracing.u_ibex_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
  endfunction
endmodule
