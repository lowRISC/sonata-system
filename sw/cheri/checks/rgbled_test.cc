// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <riscvreg.h>
#include <stdint.h>

#include <cheri.hh>

#include "../../common/defs.h"

using namespace CHERI;

struct SonataRGBLEDCtrl {
  uint32_t rgbled0;
  uint32_t rgbled1;
  uint32_t ctrl;
  uint32_t status;

  void wait_idle() volatile { while ((status & 0x1) == 0); }

  void set_rgb(uint8_t r, uint8_t g, uint8_t b, uint32_t led) volatile {
    uint32_t rgb;

    rgb = (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16);

    wait_idle();

    if (led == 0) {
      rgbled0 = rgb;
    } else if (led == 1) {
      rgbled1 = rgb;
    }
  }

  void update() volatile {
    wait_idle();
    ctrl = 0x1;
  }

  void clear() volatile {
    wait_idle();
    ctrl = 0x2;
  }
};

void busy_wait(uint64_t cycles) {
  uint64_t end = rdcycle64() + cycles;
  while (rdcycle64() < end);
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  Capability<volatile SonataRGBLEDCtrl> rgbled_ctrl = root.cast<volatile SonataRGBLEDCtrl>();
  rgbled_ctrl.address()                             = RGBLED_CTRL_ADDRESS;
  rgbled_ctrl.bounds()                              = RGBLED_CTRL_BOUNDS;

  while (1) {
    rgbled_ctrl->set_rgb(32, 0, 0, 1);
    rgbled_ctrl->set_rgb(0, 32, 0, 0);
    rgbled_ctrl->update();

    busy_wait(20000000);

    rgbled_ctrl->set_rgb(0, 32, 0, 1);
    rgbled_ctrl->set_rgb(0, 0, 32, 0);
    rgbled_ctrl->update();

    busy_wait(20000000);

    rgbled_ctrl->set_rgb(0, 0, 32, 1);
    rgbled_ctrl->set_rgb(32, 0, 0, 0);
    rgbled_ctrl->update();

    busy_wait(20000000);

    rgbled_ctrl->clear();

    busy_wait(20000000);
  }
}
