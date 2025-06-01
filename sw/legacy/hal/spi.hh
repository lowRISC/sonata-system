// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "mmio/spi.hh"
#include "platform.hh"

class Spi {
  const platform::Spi base_addr;

 public:
  constexpr Spi(platform::Spi base_addr, const bool clock_polarity = true, const bool clock_phase = true,
                const bool msb_first = true, const uint16_t clock_divider = 0)
      : base_addr(base_addr) {
    mmio::spi::Spi spi(base_addr);

    spi.cfg.cpol.bit(clock_polarity);
    spi.cfg.cpha.bit(clock_phase);
    spi.cfg.msb_first.bit(msb_first);
    spi.cfg.half_clk_period.write(clock_divider);
    spi.cfg.commit();

    spi.ctrl.tx_clear.set();
    spi.ctrl.rx_clear.set();
    spi.ctrl.sw_reset.set();
    spi.ctrl.commit();
  }

  bool blocking_write(const uint8_t array[], std::size_t size) {
    if (size == 0) {
      return false;
    }

    mmio::spi::Spi spi(base_addr);
    spi.ctrl.fetch().tx_enable.set().commit();
    spi.start.byte_count.write(size).commit();

    for (std::size_t i = 0, available = 0; i < size; ++i) {
      while (available == 0) {
        available = 8 - spi.status.fetch().tx_fifo_level.get();
      }
      spi.tx_fifo.data.write(array[i]).commit();
      available--;
    }
    while (!spi.status.fetch().idle.is_set()) {
    };
    return true;
  }

  bool blocking_read(uint8_t array[], std::size_t size) {
    mmio::spi::Spi base(base_addr);
    if (size == 0 || size > base.start.byte_count.max()) {
      return false;
    }

    base.ctrl.fetch().rx_enable.set().commit();
    base.start.byte_count.write(size).commit();

    for (std::size_t i = 0; i < size; ++i) {
      while (base.status.fetch().rx_fifo_level.get() > 0);
      array[i] = base.rx_fifo.fetch().data.get();
    }
    return true;
  }

  template <std::size_t CS, bool DEASSERT_OTHERS = true>
  inline void chip_select_assert(bool assert = true) {
    mmio::spi::Spi spi(base_addr);
    spi.cs.fetch();
    if constexpr (DEASSERT_OTHERS) {
      spi.cs.cs.write(spi.cs.cs.max());
    }
    spi.cs.cs.bit_mask(!assert, CS).commit();
  }
};

class Lcd : public Spi {
 public:
  enum ChipSelect : std::size_t {
    Cs      = 0,
    Command = 1,
    Reset   = 2,
  };

  inline void chip_select(bool assert) { this->Spi::chip_select_assert<ChipSelect::Cs, false>(assert); }

  inline void command_assert(bool assert) { this->Spi::chip_select_assert<ChipSelect::Command, false>(!assert); }

  inline void reset(bool assert) { this->Spi::chip_select_assert<ChipSelect::Reset, false>(assert); }
};
