/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "asm.hh"

extern "C" __attribute__((alias("exception_handler"))) __attribute__((weak)) void irq_software_handler(void);

extern "C" __attribute__((weak)) void irq_timer_handler(void) { return; }

extern "C" __attribute__((alias("exception_handler"))) __attribute__((weak)) void irq_external_handler(void);

extern "C" __attribute__((alias("exception_handler"))) __attribute__((weak)) void irq_internal_handler(void);

extern "C" __attribute__((weak)) void exception_handler(void) { while (1); }

enum ExceptionCode : uint32_t {
  MachineSoftwareInterrupt = 3,
  MachineTimerInterrupt    = 7,
  MachineExternalInterrupt = 11,
};

extern "C" void __trap_vector(void) {
  uint32_t mcause = READ_CSR("mcause");
  if (mcause == 0x1c || mcause < 0x08) {
    exception_handler();
  }

  switch (mcause & 0x1f) {
    case MachineSoftwareInterrupt:
      irq_software_handler();
      break;
    case MachineTimerInterrupt:
      irq_timer_handler();
      break;
    case MachineExternalInterrupt:
      irq_external_handler();
      break;
    default:
      break;
  }
}
