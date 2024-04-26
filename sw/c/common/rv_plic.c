// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "rv_plic.h"

#include "sonata_system.h"
#include "dev_access.h"

#define NUM_IRQS 182

#define RV_PLIC_IRQ_PRIO_BASE_REG    0x000000
#define RV_PLIC_IRQ_PENDING_BASE_REG 0x001000
#define RV_PLIC_IRQ_ENABLE_BASE_REG  0x002000
#define RV_PLIC_CTX_THRESHOLD_REG    0x200000
#define RV_PLIC_CTX_CLAIM_REG        0x200004
#define RV_PLIC_CTX_COMPLETION_REG   0x200004

static irq_handler_t irq_handlers[NUM_IRQS];

static void rv_plic_handler(void) __attribute__((interrupt));

static void rv_plic_handler(void) {
  while (true) {
    irq_t claim = DEV_READ(RV_PLIC_BASE + RV_PLIC_CTX_CLAIM_REG);
    if (!claim) return;

    irq_handlers[claim](claim);

    DEV_WRITE(RV_PLIC_BASE + RV_PLIC_CTX_COMPLETION_REG, claim);
  }
}

void rv_plic_init(void) {
  // Set all irq priority to 0.
  for (size_t i = 1; i < NUM_IRQS; i++) {
    DEV_WRITE(RV_PLIC_BASE + RV_PLIC_IRQ_PRIO_BASE_REG + 4 * i, 0);
  }
  // Disable all IRQs.
  for (size_t i = 0; i < (NUM_IRQS + 31) / 32; i++) {
    DEV_WRITE(RV_PLIC_BASE + RV_PLIC_IRQ_ENABLE_BASE_REG + 4 * i, 0);
  }
  // Set priority threshold to 0 so IRQs can trigger with any non-zero priority.
  DEV_WRITE(RV_PLIC_BASE + RV_PLIC_CTX_THRESHOLD_REG, 0);

  install_exception_handler(11, rv_plic_handler);
  enable_interrupts(1 << 11);
  set_global_interrupt_enable(1);
}

void rv_plic_register_irq(irq_t irq, irq_handler_t handler) {
  irq_handlers[irq] = handler;
}

bool rv_plic_pending(irq_t irq) {
  uint32_t addr = RV_PLIC_BASE + RV_PLIC_IRQ_PENDING_BASE_REG + irq / 32 * 4;
  uint32_t mask = 1 << (irq % 32);
  return (DEV_READ(addr) & mask) != 0;
}

void rv_plic_enable(irq_t irq) {
  // Set priority to 1.
  DEV_WRITE(RV_PLIC_BASE + RV_PLIC_IRQ_PRIO_BASE_REG + 4 * irq, 1);
  // Set enable bit to 1.
  uint32_t addr = RV_PLIC_BASE + RV_PLIC_IRQ_ENABLE_BASE_REG + irq / 32 * 4;
  uint32_t mask = 1 << (irq % 32);
  uint32_t reg = DEV_READ(addr);
  reg |= mask;
  DEV_WRITE(addr, reg);
}

void rv_plic_disable(irq_t irq) {
  uint32_t addr = RV_PLIC_BASE + RV_PLIC_IRQ_ENABLE_BASE_REG + irq / 32 * 4;
  uint32_t mask = 1 << (irq % 32);
  uint32_t reg = DEV_READ(addr);
  reg &= ~mask;
  DEV_WRITE(addr, reg);
  DEV_WRITE(RV_PLIC_BASE + RV_PLIC_IRQ_PRIO_BASE_REG + 4 * irq, 0);
}
