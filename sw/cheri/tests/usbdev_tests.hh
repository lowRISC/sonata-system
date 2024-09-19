// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "../../common/defs.h"
#include "../common/console-utils.hh"
#include "../common/sonata-devices.hh"
#include "../common/timer-utils.hh"
#include "../common/uart-utils.hh"
#include "../common/usbdev-utils.hh"
#include "test_runner.hh"
#include <cheri.hh>
#include <platform-uart.hh>

using namespace CHERI;

/*
 * Configures the number of test iterations to perform.
 * This can be overridden via a compilation flag
 */
#ifndef USBDEV_TEST_ITERATIONS
#define USBDEV_TEST_ITERATIONS 1U
#endif

/**
 * The timeout window within which we wait for the USB connection to configure
 * properly. We wait for a little over 500 ms.
 */
#define USBDEV_CONFIGURE_TIMEOUT_CYCLES 500U * 70000U

// Device Descriptor; see "Table 9-8. Standard Device Descriptor"
static const uint8_t _Alignas(uint32_t) deviceDescriptor[] = {
    0x12u, 1, 0, 2, 0, 0, 0, OpenTitanUsbdev::MaxPacketLen, 0xd1, 0x18, 0x3a, 0x50, 0, 1, 0, 0, 0, 1};

// Configuration Descriptor; see "Table 9-10. Standard Configuration Descriptor"
static const uint8_t _Alignas(uint32_t) configDescriptor[] = {
    // Present a single interface consisting of an IN EP and and OUT EP.
    USB_CFG_DSCR_HEAD(USB_CFG_DSCR_LEN + (USB_INTERFACE_DSCR_LEN + 2 * USB_EP_DSCR_LEN), 1),
    VEND_INTERFACE_DSCR(0, 2, 0x50, 1),
    USB_BULK_EP_DSCR(0, 1, OpenTitanUsbdev::MaxPacketLen, 0),
    USB_BULK_EP_DSCR(1, 1, OpenTitanUsbdev::MaxPacketLen, 4),
};

// Default test descriptor; required by the USB DPI model and retrieved using a
// Vendor Specific Control Transfer. Real USB host controllers will not retrieve
// this descriptor.
static const uint8_t _Alignas(uint32_t) testDescriptor[] = {USB_TESTUTILS_TEST_DSCR(0, 0, 0, 0, 0)};

/**
 * Test whether we can successfully configure the USB device, setting up the
 * IN/OUT endpoint pair 1, connecting, and configuring the USB device within
 * a given timeout, before finally disconnecting the USB device.
 */
static int usbdev_configure_test(Capability<volatile OpenTitanUsbdev> usbdev) {
  int failures = 0;

  // Initialise the handling of standard Control Transfer requests on the
  // Default Control Pipe
  UsbdevUtils usb(usbdev, deviceDescriptor, sizeof(deviceDescriptor), configDescriptor, sizeof(configDescriptor),
                  testDescriptor, sizeof(testDescriptor));

  // Configure the IN/OUT endpoint pair 1 to act as a simple serial port
  // No RX callback is defined for this test.
  bool ok = usb.setup_out_endpoint(1u, true, false, false, nullptr, nullptr);
  if (ok) ok = usb.setup_in_endpoint(1u, true, false, nullptr, nullptr);
  if (!ok) failures++;

  // Connect to the USB and indicate our presence to the USB host controller
  ok = usb.connect();
  if (!ok) failures++;

  // Wait for the USB device to be configured.
  uint32_t configure_start_time = 0, configure_end_time = 0;
  bool usb_configured = false;
  while (configure_end_time <= configure_start_time) {  // Avoid possible overflow in mcycle.
    configure_start_time = get_mcycle();
    configure_end_time   = configure_start_time + USBDEV_CONFIGURE_TIMEOUT_CYCLES;
  }
  while (get_mcycle() < configure_end_time) {
    usb.service();
    if (usb.configured()) {
      usb_configured = true;
      break;
    }
  }

  // Check if the USB configuration timed out.
  if (!usb_configured) failures++;

  // Disconnect at the end of the test
  if (usbdev->connected()) {
    int return_code = usbdev->disconnect();
    if (return_code) failures++;
  }

  return failures;
}

/**
 * Run the whole suite of USB device tests.
 */
void usbdev_tests(CapRoot root, UartPtr console) {
  // Create a bounded capability to the USB Device
  UsbdevPtr usbdev = usbdev_ptr(root);

  // Reset mcycle to use for timing in the tests
  reset_mcycle();

  // Execute the specified number of iterations of each test
  for (size_t i = 0; i < USBDEV_TEST_ITERATIONS; i++) {
    write_str(console, "\n\nrunning usbdev_test: ");
    write_hex8b(console, i);
    write_str(console, "\\");
    write_hex8b(console, USBDEV_TEST_ITERATIONS - 1);
    write_str(console, "\r\n\x1b[35m(needs User USB connected to a Type-A USB host port)");
    write_str(console, "\x1b[0m\r\n  ");

    int failures = 0;
    write_str(console, "Running USBDEV configure test... ");
    failures = usbdev_configure_test(usbdev);
    write_test_result(console, failures);

    check_result(console, failures == 0);
  }
}
