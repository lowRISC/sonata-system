// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// sonata package

package sonata_pkg;

  // Number of Instances
  localparam int unsigned GPIO_NUM = 5;
  localparam int unsigned PWM_NUM = 1;
  localparam int unsigned UART_NUM = 3;
  localparam int unsigned I2C_NUM = 2;
  localparam int unsigned SPI_NUM = 3;

  // Width of block IO arrays
  localparam int unsigned GPIO_IOS_WIDTH = 32;
  localparam int unsigned PWM_OUT_WIDTH = 7;
  localparam int unsigned SPI_CS_WIDTH = 4;

  // Number of input, output, and inout pins
  localparam int unsigned IN_PIN_NUM = 8;
  localparam int unsigned OUT_PIN_NUM = 15;
  localparam int unsigned INOUT_PIN_NUM = 70;

  localparam int unsigned IN_PIN_SER0_RX = 0;
  localparam int unsigned IN_PIN_SER1_RX = 1;
  localparam int unsigned IN_PIN_RS232_RX = 2;
  localparam int unsigned IN_PIN_RS485_RX = 3;
  localparam int unsigned IN_PIN_MB3 = 4;
  localparam int unsigned IN_PIN_MB8 = 5;
  localparam int unsigned IN_PIN_APPSPI_D1 = 6;
  localparam int unsigned IN_PIN_MICROSD_DAT0 = 7;

  localparam int unsigned OUT_PIN_SER0_TX = 0;
  localparam int unsigned OUT_PIN_SER1_TX = 1;
  localparam int unsigned OUT_PIN_RS232_TX = 2;
  localparam int unsigned OUT_PIN_RS485_TX = 3;
  localparam int unsigned OUT_PIN_MB1 = 4;
  localparam int unsigned OUT_PIN_MB2 = 5;
  localparam int unsigned OUT_PIN_MB4 = 6;
  localparam int unsigned OUT_PIN_MB7 = 7;
  localparam int unsigned OUT_PIN_MB10 = 8;
  localparam int unsigned OUT_PIN_APPSPI_D0 = 9;
  localparam int unsigned OUT_PIN_APPSPI_CLK = 10;
  localparam int unsigned OUT_PIN_APPSPI_CS = 11;
  localparam int unsigned OUT_PIN_MICROSD_CMD = 12;
  localparam int unsigned OUT_PIN_MICROSD_CLK = 13;
  localparam int unsigned OUT_PIN_MICROSD_DAT3 = 14;

  localparam int unsigned INOUT_PIN_SCL0 = 0;
  localparam int unsigned INOUT_PIN_SDA0 = 1;
  localparam int unsigned INOUT_PIN_SCL1 = 2;
  localparam int unsigned INOUT_PIN_SDA1 = 3;
  localparam int unsigned INOUT_PIN_RPH_G0 = 4;
  localparam int unsigned INOUT_PIN_RPH_G1 = 5;
  localparam int unsigned INOUT_PIN_RPH_G2_SDA = 6;
  localparam int unsigned INOUT_PIN_RPH_G3_SCL = 7;
  localparam int unsigned INOUT_PIN_RPH_G4 = 8;
  localparam int unsigned INOUT_PIN_RPH_G5 = 9;
  localparam int unsigned INOUT_PIN_RPH_G6 = 10;
  localparam int unsigned INOUT_PIN_RPH_G7 = 11;
  localparam int unsigned INOUT_PIN_RPH_G8 = 12;
  localparam int unsigned INOUT_PIN_RPH_G9 = 13;
  localparam int unsigned INOUT_PIN_RPH_G10 = 14;
  localparam int unsigned INOUT_PIN_RPH_G11 = 15;
  localparam int unsigned INOUT_PIN_RPH_G12 = 16;
  localparam int unsigned INOUT_PIN_RPH_G13 = 17;
  localparam int unsigned INOUT_PIN_RPH_TXD0 = 18;
  localparam int unsigned INOUT_PIN_RPH_RXD0 = 19;
  localparam int unsigned INOUT_PIN_RPH_G16 = 20;
  localparam int unsigned INOUT_PIN_RPH_G17 = 21;
  localparam int unsigned INOUT_PIN_RPH_G18 = 22;
  localparam int unsigned INOUT_PIN_RPH_G19 = 23;
  localparam int unsigned INOUT_PIN_RPH_G20 = 24;
  localparam int unsigned INOUT_PIN_RPH_G21 = 25;
  localparam int unsigned INOUT_PIN_RPH_G22 = 26;
  localparam int unsigned INOUT_PIN_RPH_G23 = 27;
  localparam int unsigned INOUT_PIN_RPH_G24 = 28;
  localparam int unsigned INOUT_PIN_RPH_G25 = 29;
  localparam int unsigned INOUT_PIN_RPH_G26 = 30;
  localparam int unsigned INOUT_PIN_RPH_G27 = 31;
  localparam int unsigned INOUT_PIN_AH_TMPIO0 = 32;
  localparam int unsigned INOUT_PIN_AH_TMPIO1 = 33;
  localparam int unsigned INOUT_PIN_AH_TMPIO2 = 34;
  localparam int unsigned INOUT_PIN_AH_TMPIO3 = 35;
  localparam int unsigned INOUT_PIN_AH_TMPIO4 = 36;
  localparam int unsigned INOUT_PIN_AH_TMPIO5 = 37;
  localparam int unsigned INOUT_PIN_AH_TMPIO6 = 38;
  localparam int unsigned INOUT_PIN_AH_TMPIO7 = 39;
  localparam int unsigned INOUT_PIN_AH_TMPIO8 = 40;
  localparam int unsigned INOUT_PIN_AH_TMPIO9 = 41;
  localparam int unsigned INOUT_PIN_AH_TMPIO10 = 42;
  localparam int unsigned INOUT_PIN_AH_TMPIO11 = 43;
  localparam int unsigned INOUT_PIN_AH_TMPIO12 = 44;
  localparam int unsigned INOUT_PIN_AH_TMPIO13 = 45;
  localparam int unsigned INOUT_PIN_MB5 = 46;
  localparam int unsigned INOUT_PIN_MB6 = 47;
  localparam int unsigned INOUT_PIN_PMOD0_1 = 48;
  localparam int unsigned INOUT_PIN_PMOD0_2 = 49;
  localparam int unsigned INOUT_PIN_PMOD0_3 = 50;
  localparam int unsigned INOUT_PIN_PMOD0_4 = 51;
  localparam int unsigned INOUT_PIN_PMOD0_7 = 52;
  localparam int unsigned INOUT_PIN_PMOD0_8 = 53;
  localparam int unsigned INOUT_PIN_PMOD0_9 = 54;
  localparam int unsigned INOUT_PIN_PMOD0_10 = 55;
  localparam int unsigned INOUT_PIN_PMOD1_1 = 56;
  localparam int unsigned INOUT_PIN_PMOD1_2 = 57;
  localparam int unsigned INOUT_PIN_PMOD1_3 = 58;
  localparam int unsigned INOUT_PIN_PMOD1_4 = 59;
  localparam int unsigned INOUT_PIN_PMOD1_7 = 60;
  localparam int unsigned INOUT_PIN_PMOD1_8 = 61;
  localparam int unsigned INOUT_PIN_PMOD1_9 = 62;
  localparam int unsigned INOUT_PIN_PMOD1_10 = 63;
  localparam int unsigned INOUT_PIN_PMODC_1 = 64;
  localparam int unsigned INOUT_PIN_PMODC_2 = 65;
  localparam int unsigned INOUT_PIN_PMODC_3 = 66;
  localparam int unsigned INOUT_PIN_PMODC_4 = 67;
  localparam int unsigned INOUT_PIN_PMODC_5 = 68;
  localparam int unsigned INOUT_PIN_PMODC_6 = 69;

  typedef logic [   IN_PIN_NUM-1:0] sonata_in_pins_t;
  typedef logic [  OUT_PIN_NUM-1:0] sonata_out_pins_t;
  typedef logic [INOUT_PIN_NUM-1:0] sonata_inout_pins_t;

endpackage : sonata_pkg
