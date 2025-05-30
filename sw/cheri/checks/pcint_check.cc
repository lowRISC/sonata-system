/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Interact with the Pin-Change Interrupt (PCINT) functionality of the
 * GPIO using the on-board DIP switches, joystick push-button, and LEDs.
 *
 * User DIP switches (SW4) and user LEDs
 *   US0 / ULED0: interrupt enable
 *   US1 / ULED1: mode0
 *   US2 / ULED2: mode1
 *   US3 / ULED3: debounce select
 *
 *   US6: clear irq_fired    / ULED6: irq_fired
 *   US7: clear PCINT status / ULED7: PCINT status
 *
 * SW8 (joystick)
 *   push button: input pin state -> release=low, pressed=high
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <stdint.h>

#include <cheri.hh>
#include <platform-plic.hh>
#include "../common/asm.hh"
#include "../common/sonata-devices.hh"

using namespace CHERI;

PLIC::SonataPlic *plic;
volatile bool irq_fired;
Capability<volatile uint32_t> cas;

/**
 * Overwride the default handler
 */
extern "C" void irq_external_handler(void) {
  if (auto irq = plic->interrupt_claim()) {
    *cas = *cas;  // clear interrupt at source
    plic->interrupt_complete(*irq);
    irq_fired = true;
  }
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Capability to general purpose output
  Capability<volatile uint32_t> gpo = root.cast<volatile uint32_t>();
  gpo.address()                     = 0x80000000;
  gpo.bounds()                      = sizeof(uint32_t);

  // Capability to general purpose input
  Capability<volatile uint32_t> gpi = root.cast<volatile uint32_t>();
  gpi.address()                     = 0x80000004;
  gpi.bounds()                      = sizeof(uint32_t);

  // Capability to PCINT mask
  Capability<volatile uint32_t> msk = root.cast<volatile uint32_t>();
  msk.address()                     = 0x80000010;
  msk.bounds()                      = sizeof(uint32_t);

  // Capability to (PCINT) control and status
  cas           = root.cast<volatile uint32_t>();
  cas.address() = 0x80000014;
  cas.bounds()  = sizeof(uint32_t);

  PLIC::SonataPlic _plic(root);
  plic      = &_plic;
  irq_fired = false;
  plic->interrupt_enable(PLIC::Interrupts::Pcint);
  plic->priority_set(PLIC::Interrupts::Pcint, 1);
  ASM::Ibex::external_interrupt_set(true);
  ASM::Ibex::global_interrupt_set(true);

  uint32_t inputValue  = 0;
  uint32_t switchValue = 0;
  uint32_t casValue    = 0;
  *msk                 = 0x0400;  // trigger PCINT using joystick button only
  *cas                 = 0x1;     // ensure flag is clear at start
  while (true) {
    inputValue  = *gpi;
    switchValue = inputValue & 0xFF;
    *cas        = (switchValue << 16) | (switchValue >> 7);  // control PCINT using DIP switches
    casValue    = *cas;
    *gpo        = ((casValue & 0x01) << 7) | (irq_fired << 6) | (casValue >> 16);  // display PCINT status on LEDs
    if (switchValue & (1 << 6)) irq_fired = false;
  }

  asm volatile("wfi");
}
