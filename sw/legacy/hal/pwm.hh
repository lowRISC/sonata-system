// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "mmio/pwm.hh"
#include "platform.hh"

class Pwm {
  const platform::Pwm base_addr;

 public:
  constexpr Pwm(platform::Pwm base_addr) : base_addr(base_addr) {}

  void set(uint8_t counter, uint8_t pulse_width) {
    mmio::pwm::Pwm pwm(base_addr);
    pwm.counter.value.write(counter).commit();
    pwm.width.value.write(pulse_width).commit();
  }
};
