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
#include <platform-pwm.hh>
#include <platform-uart.hh>
#include <platform-i2c.hh>
#include <platform-spi.hh>
#include <platform-usbdev.hh>
#include "platform-pinmux.hh"

typedef CHERI::Capability<void> CapRoot;
typedef volatile SonataGpioBase<> *GpioPtr;
typedef volatile SonataPulseWidthModulation::General *PwmPtr;
typedef volatile OpenTitanUart *UartPtr;
typedef volatile OpenTitanUsbdev *UsbdevPtr;
typedef volatile OpenTitanI2c *I2cPtr;
typedef volatile SonataSpi::Generic<> *SpiPtr;
typedef volatile OpenTitanUsbdev *UsbdevPtr;
typedef volatile uint32_t *HyperramPtr;
typedef volatile uint32_t *TimerPtr;
typedef PLIC::SonataPlic *PlicPtr;
typedef volatile SonataPinmux::PinSinks *PinSinksPtr;
typedef volatile SonataPinmux::BlockSinks *BlockSinksPtr;
using PinmuxPtrs = std::pair<PinSinksPtr, BlockSinksPtr>;

[[maybe_unused]] static GpioPtr gpio_ptr(CapRoot root, uint8_t index = 0) {
  CHERI::Capability<volatile SonataGpioBase<>> gpio = root.cast<volatile SonataGpioBase<>>();
  gpio.address()                                    = GPIO_ADDRESS + GPIO_RANGE * index;
  gpio.bounds()                                     = GPIO_BOUNDS;
  return gpio;
}

[[maybe_unused]] static PwmPtr pwm_ptr(CapRoot root) {
  using SonataPwm                           = SonataPulseWidthModulation::General;
  CHERI::Capability<volatile SonataPwm> pwm = root.cast<volatile SonataPwm>();
  pwm.address()                             = PWM_ADDRESS;
  pwm.bounds()                              = PWM_BOUNDS * PWM_NUM;
  return pwm;
}

[[maybe_unused]] static UartPtr uart_ptr(CapRoot root, uint32_t idx = 0) {
  CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  assert(idx < UART_NUM);
  uart.address() = UART_ADDRESS + (idx * UART_RANGE);
  uart.bounds()  = UART_BOUNDS;
  return uart;
}

[[maybe_unused]] static I2cPtr i2c_ptr(CapRoot root, uint32_t idx = 0) {
  CHERI::Capability<volatile OpenTitanI2c> i2c = root.cast<volatile OpenTitanI2c>();
  assert(idx < I2C_NUM);
  i2c.address() = I2C_ADDRESS + (idx * I2C_RANGE);
  i2c.bounds()  = I2C_BOUNDS;
  return i2c;
}

[[maybe_unused]] static SpiPtr spi_ptr(CapRoot root, uint32_t idx = 0) {
  using SonataSpi                           = SonataSpi::Generic<>;
  CHERI::Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  assert(idx < SPI_NUM);
  spi.address() = SPI_ADDRESS + (idx * SPI_RANGE);
  spi.bounds()  = SPI_BOUNDS;
  return spi;
}

[[maybe_unused]] static HyperramPtr hyperram_ptr(CapRoot root) {
  // Unfortunately it is not possible to construct a capability that covers exactly the 8MiB range
  // of the HyperRAM because of the encoding limitations of the CHERIoT capabilities.
  // We therefore leave the final 16KiB inaccessible.
  CHERI::Capability<volatile uint32_t> hyperram = root.cast<volatile uint32_t>();
  hyperram.address()                            = HYPERRAM_ADDRESS;
  hyperram.bounds()                             = HYPERRAM_BOUNDS - 0x4000u;
  return hyperram;
}

[[maybe_unused]] static TimerPtr timer_ptr(CapRoot root) {
  CHERI::Capability<volatile uint32_t> timer = root.cast<volatile uint32_t>();
  timer.address()                            = CLINT_ADDRESS;
  timer.bounds()                             = CLINT_BOUNDS;
  return timer;
}

[[maybe_unused]] static UsbdevPtr usbdev_ptr(CapRoot root) {
  CHERI::Capability<volatile OpenTitanUsbdev> usbdev = root.cast<volatile OpenTitanUsbdev>();
  usbdev.address()                                   = USBDEV_ADDRESS;
  usbdev.bounds()                                    = USBDEV_BOUNDS;
  return usbdev;
}

[[maybe_unused]] static PinSinksPtr pin_sinks_ptr(CapRoot root) {
  using namespace SonataPinmux;
  CHERI::Capability pinSinks = root.cast<volatile PinSinks>();
  pinSinks.address()         = PINMUX_PIN_SINKS_ADDRESS;
  pinSinks.bounds()          = PINMUX_PIN_SINKS_BOUNDS;
  static_assert(sizeof(PinSinks) == PINMUX_PIN_SINKS_BOUNDS);
  return pinSinks;
}

[[maybe_unused]] static BlockSinksPtr block_sinks_ptr(CapRoot root) {
  using namespace SonataPinmux;
  CHERI::Capability blockSinks = root.cast<volatile BlockSinks>();
  blockSinks.address()         = PINMUX_BLOCK_SINKS_ADDRESS;
  blockSinks.bounds()          = PINMUX_BLOCK_SINKS_BOUNDS;
  static_assert(sizeof(BlockSinks) == PINMUX_BLOCK_SINKS_BOUNDS);
  return blockSinks;
}
