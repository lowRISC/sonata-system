// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// tl_main package generated by `tlgen.py` tool

package tl_main_pkg;

  localparam logic [31:0] ADDR_SPACE_SRAM       = 32'h 00100000;
  localparam logic [31:0] ADDR_SPACE_HYPERRAM   = 32'h 40000000;
  localparam logic [31:0] ADDR_SPACE_REV_TAG    = 32'h 30000000;
  localparam logic [31:0] ADDR_SPACE_PERI       = 32'h 80000000;
  localparam logic [31:0] ADDR_SPACE_SPI_LCD    = 32'h 80300000;
  localparam logic [31:0] ADDR_SPACE_SPI_ETHMAC = 32'h 80301000;
  localparam logic [31:0] ADDR_SPACE_I2C0       = 32'h 80200000;
  localparam logic [31:0] ADDR_SPACE_I2C1       = 32'h 80201000;
  localparam logic [31:0] ADDR_SPACE_SPI0       = 32'h 80302000;
  localparam logic [31:0] ADDR_SPACE_SPI1       = 32'h 80303000;
  localparam logic [31:0] ADDR_SPACE_SPI2       = 32'h 80304000;
  localparam logic [31:0] ADDR_SPACE_USBDEV     = 32'h 80400000;
  localparam logic [31:0] ADDR_SPACE_RV_PLIC    = 32'h 88000000;

  localparam logic [31:0] ADDR_MASK_SRAM       = 32'h 0003ffff;
  localparam logic [31:0] ADDR_MASK_HYPERRAM   = 32'h 000fffff;
  localparam logic [31:0] ADDR_MASK_REV_TAG    = 32'h 00003fff;
  localparam logic [31:0] ADDR_MASK_PERI       = 32'h 001fffff;
  localparam logic [31:0] ADDR_MASK_SPI_LCD    = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI_ETHMAC = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_I2C0       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_I2C1       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI0       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI1       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI2       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_USBDEV     = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_RV_PLIC    = 32'h 03ffffff;

  localparam int N_HOST   = 2;
  localparam int N_DEVICE = 13;

  typedef enum int {
    TlSram = 0,
    TlHyperram = 1,
    TlRevTag = 2,
    TlPeri = 3,
    TlSpiLcd = 4,
    TlSpiEthmac = 5,
    TlI2C0 = 6,
    TlI2C1 = 7,
    TlSpi0 = 8,
    TlSpi1 = 9,
    TlSpi2 = 10,
    TlUsbdev = 11,
    TlRvPlic = 12
  } tl_device_e;

  typedef enum int {
    TlIbexLsu = 0,
    TlDbgHost = 1
  } tl_host_e;

endpackage
