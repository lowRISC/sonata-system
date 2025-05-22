// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "mmio/timer.hh"
#include "platform.hh"

class Timer {
 public:
  const platform::Timer base_addr;
  const std::size_t clock_hz;

  constexpr Timer(platform::Timer base_addr, std::size_t clock) : base_addr(base_addr), clock_hz(clock) {
    mmio::timer::Timer timer(base_addr);
    timer.mtimecmph.value.write(std::numeric_limits<uint32_t>::max()).commit();
    timer.mtimecmpl.value.write(std::numeric_limits<uint32_t>::max()).commit();
  }

  uint64_t get() {
    mmio::timer::Timer timer(base_addr);
    return static_cast<uint64_t>(timer.mtimeh.fetch().value.get()) << 32 | timer.mtimel.fetch().value.get();
  }

  void delay(uint64_t millis) {
    uint64_t timeout = (millis * clock_hz / 1000) + get();
    while (timeout > get());
  }
};

class CountDown {
  Timer& timer;
  uint64_t timeout;

 public:
  CountDown(Timer& timer, uint64_t millis) : timer(timer) { timeout = (millis * timer.clock_hz / 1000) + timer.get(); }

  bool has_timeout() { return timeout < timer.get(); }

  void wait() { while (!has_timeout()); }
};
