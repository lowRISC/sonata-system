/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>

namespace ASM {
// This is using GNU statement expression extension so we can return a value
// from a macro: https://gcc.gnu.org/onlinedocs/gcc/Statement-Exprs.html
#define READ_CSR(name)                                                                                                 \
  ({                                                                                                                   \
    uint32_t result;                                                                                                   \
    asm volatile("csrr %0, " name : "=r"(result));                                                                     \
    result;                                                                                                            \
  })

#define WRITE_CSR(name, value) ({ asm volatile("csrw " name ", %0 " ::"r"(value)); })

struct Ibex {
  static void nop(void) { __asm__ volatile("nop"); }
  static void wfi(void) { __asm__ volatile("wfi"); }

  static void global_interrupt_set(bool enable) {
    uint32_t mstatus = READ_CSR("mstatus");
    mstatus &= ~(enable << 3);
    mstatus |= enable << 3;
    WRITE_CSR("mstatus", mstatus);
  }

  static void external_interrupt_set(bool enable) {
    uint32_t mie = READ_CSR("mie");
    mie &= ~(enable << 11);
    mie |= enable << 11;
    WRITE_CSR("mie", mie);
  }

  static void timer_interrupt_set(bool enable) {
    uint32_t mie = READ_CSR("mie");
    mie &= ~(enable << 7);
    mie |= enable << 7;
    WRITE_CSR("mie", mie);
  }
};
};  // namespace ASM
