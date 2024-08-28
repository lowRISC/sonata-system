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

#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-uart.hh>

typedef CHERI::Capability<void> CapRoot;
typedef volatile SonataGPIO *GpioPtr;
typedef volatile OpenTitanUart *UartPtr;
typedef volatile uint32_t *HyperramPtr;
typedef PLIC::SonataPlic *PlicPtr;

[[maybe_unused]] static GpioPtr gpio_ptr(CapRoot root) {
  CHERI::Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                              = GPIO_ADDRESS;
  gpio.bounds()                               = GPIO_BOUNDS;
  return gpio;
}

[[maybe_unused]] static UartPtr uart_ptr(CapRoot root, uint32_t uart_idx = 0) {
  CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  switch (uart_idx) {
    case 1:
      uart.address() = UART1_ADDRESS;
      break;
    default:
      uart.address() = UART_ADDRESS;
  };
  uart.bounds() = UART_BOUNDS;
  return uart;
};

[[maybe_unused]] static HyperramPtr hyperram_ptr(CapRoot root) {
  CHERI::Capability<volatile uint32_t> hyperram = root.cast<volatile uint32_t>();
  hyperram.address()                            = HYPERRAM_ADDRESS;
  hyperram.bounds()                             = HYPERRAM_BOUNDS;
  return hyperram;
}
