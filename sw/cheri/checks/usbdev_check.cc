/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../common/uart-utils.hh"
#include "../common/usbdev-utils.hh"

#include <platform-uart.hh>

#define LOG(...) write_str(uart, __VA_ARGS__)

using namespace CHERI;

// Shall we disconnect once we've been configured?
//
// - configuration indicates successful communication over the USB and the ability to send and
//   receive data. For an FPGA, however, the host may be monitoring the USB serial connection
//   (eg. /dev/ttyUSBx) and it may be useful to keep running and transmitting text.
//
// - if disconnection is not enabled then a terminator emulator may be attached to `/dev/ttyUSB<n>`
//   and a sign-on message should be observed. Any characters entered into the terminal will
//   be echoed on the UART output.
static constexpr bool do_disconnect = true;

static void write_strn(UartPtr uart, const char *str, size_t len) {
  while (len-- > 0u) {
    uart->blocking_write(*str);
    ++str;
  }
}

// Sign-on text after the USB device has been successfully configured.
// - this can be observed on a USB serial port of a physical host, eg. 'cat /dev/ttyUSB<>'
static const uint8_t _Alignas(uint32_t) signon[] = "Hello from CHERI USB!\r\n";

// Device Descriptor; see "Table 9-8. Standard Device Descriptor"
static const uint8_t _Alignas(uint32_t) dev_dscr[] = {
    0x12u, 1, 0, 2, 0, 0, 0, OpenTitanUsbdev::MaxPacketLength, 0xd1, 0x18, 0x3a, 0x50,  // Google lowRISC generic FS USB
    0,     1, 0, 0, 0, 1};

// Configuration Descriptor; see "Table 9-10. Standard Configuration Descriptor"
static const uint8_t _Alignas(uint32_t) cfg_dscr[] = {
    // Present a single interface consisting of an IN EP and and OUT EP.
    USB_CFG_DSCR_HEAD(USB_CFG_DSCR_LEN + (USB_INTERFACE_DSCR_LEN + 2 * USB_EP_DSCR_LEN), 1),
    VEND_INTERFACE_DSCR(0, 2, 0x50, 1),
    USB_BULK_EP_DSCR(0, 1, OpenTitanUsbdev::MaxPacketLength, 0),
    USB_BULK_EP_DSCR(1, 1, OpenTitanUsbdev::MaxPacketLength, 4),
};

// Default test descriptor; required by the USB DPI model and retrieved using a Vendor Specific
// Control Transfer. Real USB host controllers will not retrieve this descriptor.
static const uint8_t _Alignas(uint32_t) test_dscr[] = {USB_TESTUTILS_TEST_DSCR(0, 0, 0, 0, 0)};

// Packet reception callback for endpoint 1
static void rxCallback(void *rxHandle, uint8_t ep, bool setup, const uint8_t *data, uint16_t pktLen) {
  Capability<volatile OpenTitanUart> *uart;
  uart = reinterpret_cast<Capability<volatile OpenTitanUart> *>(rxHandle);
  // Simply report any text to the UART output.
  write_strn(*uart, reinterpret_cast<const char *>(data), pktLen);
}

extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  // Buffer for data transfer to/from the USB device.
  //  uint8_t _Alignas(uint32_t) data[OpenTitanUsbdev::MaxPacketLength];

  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  Capability<volatile OpenTitanUsbdev> usbdev = root.cast<volatile OpenTitanUsbdev>();
  usbdev.address()                            = USBDEV_ADDRESS;
  usbdev.bounds()                             = USBDEV_BOUNDS;

  uart->init(BAUD_RATE);

  LOG("Initialising USB\r\n");

  // Initialise the handling of the standard Control Transfer requests on the Default Control Pipe.
  UsbdevUtils usb(usbdev, dev_dscr, sizeof(dev_dscr), cfg_dscr, sizeof(cfg_dscr), test_dscr, sizeof(test_dscr));

  // Configure IN/OUT endpoint pair 1 to act as a simple serial port.
  bool ok = usb.setup_out_endpoint(1u, true, false, false, rxCallback, &uart);
  if (ok) ok = usb.setup_in_endpoint(1u, true, false, nullptr, nullptr);
  assert(ok);

  // Connect to the USB and indicate our presence to the USB host controller.
  ok = usb.connect();
  assert(ok);

  // Note: be very careful about the use of UART-based logging after this point because neither the
  //       USB DPI model nor a physical USB host controller will wait around/retry transfers
  //       indefinitely and UART traffic can easily block the CPU waiting for FIFO space.
  LOG("Connected\r\n");  // A brief progress indicator.

  bool sent = false;
  while (true) {
    usb.service();

    // Has the USB device been configured yet?
    if (usb.configured()) {
      // Disconnect now that we've been configured successfully?
      if (do_disconnect) {
        if (usbdev->connected()) {
          // Disconnect from the USB/host controller.
          int rc = usbdev->disconnect();
          assert(!rc);
          LOG("Test passed; disconnected from USB.\r\n");
        }
      } else if (!sent) {
        // Remain connected; send a sign-on message, and then echo any input from the simpleserial
        // connection on the UART output.
        if (usb.send_data(1U, reinterpret_cast<const uint32_t *>(signon), sizeof(signon))) {
          LOG("Sent sign-on message over USB.");
          sent = true;
        } else {
          // Packet could not be sent; notify via the UART.
          LOG("Unable to send sign-on message over the USB.");
        }
      }
    }
  }
}
