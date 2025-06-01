// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "mmio/uart.hh"
#include "platform.hh"

class Uart {
  const platform::Uart base_addr;

 public:
  constexpr Uart(platform::Uart base_addr, std::size_t sys_freq, std::size_t baud = 921600) : base_addr(base_addr) {
    mmio::uart::Uart base(base_addr);
    std::size_t nco = (std::size_t)((static_cast<uint64_t>(baud) << 20) / sys_freq);
    base.ctrl.nco.write(nco);
    base.ctrl.tx.set();
    base.ctrl.rx.set();
    base.ctrl.commit();
  }

  uint8_t read_byte() {
    mmio::uart::Uart base(base_addr);
    return base.rdata.fetch().value.get();
  }

  void write_byte(uint8_t c) {
    mmio::uart::Uart base(base_addr);
    while (base.status.fetch().txfull.is_set()) {
    };
    base.wdata.value.write(static_cast<std::size_t>(c)).commit();
  }
};
