/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <stdint.h>

#include <cheri.hh>

using namespace CHERI;

#define GPIO_VALUE (0x000000FF)

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // TODO fix illegal instruction exception here.
  // asm volatile(
  //     // Store capability to stack
  //     "csc              cra, -4(csp)\n"
  //     // Load capability from stack
  //     "clc              cra, -4(csp)\n"
  //     // Clear capability from stack
  //     "csc              cnull, -4(csp)\n" ::
  //         : "memory");

  // Capability to general purpose output
  Capability<volatile uint32_t> gpo = root.cast<volatile uint32_t>();
  gpo.address()                     = 0x80000000;
  gpo.bounds()                      = sizeof(uint32_t);

  // Capability to general purpose input
  Capability<volatile uint32_t> gpi = root.cast<volatile uint32_t>();
  gpi.address()                     = 0x80000004;
  gpi.bounds()                      = sizeof(uint32_t);

  // Use pointer to flash LEDs
  uint32_t gpioValue     = 0;
  uint32_t inputValue    = 0;
  uint32_t joystickValue = 0;
  uint32_t switchValue   = 0;
  while (true) {
    gpioValue ^= GPIO_VALUE;
    for (int i = 0; i < 2000000; i++) {
      inputValue                  = *((volatile uint32_t *)gpi);
      switchValue                 = inputValue & 0xFF;
      joystickValue               = (inputValue >> 8) & 0x1F;
      *((volatile uint32_t *)gpo) = (gpioValue ^ switchValue ^ joystickValue) & GPIO_VALUE;
    }
  }
}
