// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "gpio.h"
#include "pwm.h"
#include "sonata_system.h"
#include "timer.h"
#include "uart.h"
#include "usbdev.h"

// Device Descriptor; see "Table 9-8. Standard Device Descriptor"
static uint8_t dev_dscr[] = {0x12u, 1,    0,    2,    0, 0, 0, USBDEV_MAX_PACKET_LEN,
                             0xd1,  0x18, 0x3a, 0x50,  // Google lowRISC generic FS USB
                             0,     1,    0,    0,    0, 1};

// Configuration Descriptor; see "Table 9-10. Standard Configuration Descriptor"
static uint8_t cfg_dscr[] = {
    // Present a single interface consisting of an IN EP and and OUT EP.
    USB_CFG_DSCR_HEAD(USB_CFG_DSCR_LEN + (USB_INTERFACE_DSCR_LEN + 2 * USB_EP_DSCR_LEN), 1),
    VEND_INTERFACE_DSCR(0, 2, 0x50, 1),
    USB_BULK_EP_DSCR(0, 1, USBDEV_MAX_PACKET_LEN, 0),
    USB_BULK_EP_DSCR(1, 1, USBDEV_MAX_PACKET_LEN, 4),
};

// Default test descriptor, required by USBDPI model.
static uint8_t test_dscr[] = {USB_TESTUTILS_TEST_DSCR(0, 0, 0, 0, 0)};

// Single USB device present.
static usbdev_state_t usbdev;

int main(void) {
  // Endpoint used for sending messages.
  const uint8_t ep = 1u;

  uart_init(DEFAULT_UART);

  puts("hello_usbdev demo application");

  // Initialize the USB device.
  usbdev_init(&usbdev, DEFAULT_USBDEV, dev_dscr, sizeof(dev_dscr), cfg_dscr, sizeof(cfg_dscr), test_dscr,
              USB_TESTUTILS_TEST_DSCR_LEN);

  usbdev_ep_config(&usbdev, ep, true, true, false);

  while (1) {
    // Service the USB device; this receives configuration Control Transfers/
    // from the USB host and manages the release of packet buffers.
    usbdev_service(&usbdev);

    if (usbdev_active(&usbdev)) {
      // Simple activity indicator.
      int last_elapsed_time = get_mcycle();

      // Print this to USB (use the screen command to see it at /dev/ttyUSBx).
      int buf = usbdev_buf_alloc(&usbdev);
      if (buf >= 0) {
        static char _Alignas(uint32_t) msg[USBDEV_MAX_PACKET_LEN];
        uint32_t in_val  = read_gpio(GPIO_IN_DBNC);
        const char *emsg = msg + sizeof(msg);
        char *mp         = msg;
        memcpy(mp, "Hello USB! ", 11);
        mp += 11;
        mp += snputhexn(mp, emsg - mp, last_elapsed_time, 8);
        memcpy(mp, "   Input Value: ", 16);
        mp += 16;
        mp += snputhexn(mp, emsg - mp, in_val, 4);
        // 'screen' wants CR,LF termination by default.
        if (mp < emsg) {
          *mp++ = '\r';
        }
        if (mp < emsg) {
          *mp++ = '\n';
        }
        (void)usbdev_packet_send(&usbdev, ep, buf, (uint8_t *)msg, mp - msg);
      }
    }
  }
}
