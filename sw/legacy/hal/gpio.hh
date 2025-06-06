// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "mmio/gpio.hh"
#include "platform.hh"

class Gpio {
  const platform::Gpio base_addr;

 public:
  constexpr Gpio(platform::Gpio base_addr) : base_addr(base_addr) {}

  void write(uint32_t pins) {
    mmio::gpio::Gpio gpio(base_addr);
    gpio.out.pins.write(pins).commit();
  }

  uint32_t read() {
    mmio::gpio::Gpio gpio(base_addr);
    return gpio.in.fetch().pins.get();
  }

  uint32_t read_debounce() {
    mmio::gpio::Gpio gpio(base_addr);
    return gpio.in_dbnc.fetch().value.get();
  }
};
