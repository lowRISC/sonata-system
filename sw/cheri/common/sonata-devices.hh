/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
// clang-format off
#include "../../common/defs.h"
#include "sonata_plic.hh"
// clang-format on

#include <assert.h>

#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-uart.hh>
#include "../common/platform-usbdev.hh"
#include <platform-i2c.hh>
#include <platform-spi.hh>
#include "platform-usbdev.hh"

typedef CHERI::Capability<void> CapRoot;
typedef volatile SonataGPIO *GpioPtr;
typedef volatile OpenTitanUart *UartPtr;
typedef volatile OpenTitanUsbdev *UsbdevPtr;
typedef volatile OpenTitanI2c *I2cPtr;
typedef volatile SonataSpi *SpiPtr;
typedef volatile OpenTitanUsbdev *UsbdevPtr;
typedef volatile uint32_t *HyperramPtr;
typedef PLIC::SonataPlic *PlicPtr;

[[maybe_unused]] static GpioPtr gpio_ptr(CapRoot root) {
  CHERI::Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                              = GPIO_ADDRESS;
  gpio.bounds()                               = GPIO_BOUNDS;
  return gpio;
}

[[maybe_unused]] static UartPtr uart_ptr(CapRoot root, uint32_t idx = 0) {
  CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  assert(idx < UART_NUM);
  uart.address() = UART_ADDRESS + (idx * UART_RANGE);
  uart.bounds() = UART_BOUNDS;
  return uart;
}

[[maybe_unused]] static I2cPtr i2c_ptr(CapRoot root, uint32_t idx = 0) {
  CHERI::Capability<volatile OpenTitanI2c> i2c = root.cast<volatile OpenTitanI2c>();
  assert(idx < I2C_NUM);
  i2c.address() = I2C_ADDRESS + (idx * I2C_RANGE);
  i2c.bounds() = I2C_BOUNDS;
  return i2c;
}

[[maybe_unused]] static SpiPtr spi_ptr(CapRoot root, uint32_t idx = 0) {
  CHERI::Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  assert(idx < SPI_NUM);
  spi.address() = SPI_ADDRESS + (idx * SPI_RANGE);
  spi.bounds() = UART_BOUNDS;
  return spi;
}

[[maybe_unused]] static UsbdevPtr usbdev_ptr(CapRoot root) {
  CHERI::Capability<volatile OpenTitanUsbdev> usbdev = root.cast<volatile OpenTitanUsbdev>();
  usbdev.address() = USBDEV_ADDRESS;
  usbdev.bounds() = USBDEV_BOUNDS;
  return usbdev;
}

[[maybe_unused]] static HyperramPtr hyperram_ptr(CapRoot root) {
  CHERI::Capability<volatile uint32_t> hyperram = root.cast<volatile uint32_t>();
  hyperram.address()                            = HYPERRAM_ADDRESS;
  hyperram.bounds()                             = HYPERRAM_BOUNDS;
  return hyperram;
}

[[maybe_unused]] static UsbdevPtr usbdev_ptr(CapRoot root) {
  CHERI::Capability<volatile OpenTitanUsbdev> usbdev = root.cast<volatile OpenTitanUsbdev>();
  usbdev.address()                                   = USBDEV_ADDRESS;
  usbdev.bounds()                                    = USBDEV_BOUNDS;
  return usbdev;
}
