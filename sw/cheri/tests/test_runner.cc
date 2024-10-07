// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "test_runner.hh"

// clang-format off
#include "../../common/defs.h"
#include "../common/console.hh"
#include "uart_tests.hh"
#include "hyperram_tests.hh"
#include "plic_tests.hh"
#include "../common/uart-utils.hh"
#include "../common/sonata-devices.hh"
#include <cheri.hh>
// clang-format on
#include <platform-uart.hh>

#include "../common/uart-utils.hh"
#include "hyperram_tests.hh"
#include "i2c_tests.hh"
#include "spi_tests.hh"
#include "pinmux_tests.hh"
#include "uart_tests.hh"
#include "usbdev_tests.hh"

extern "C" void entry_point(void *rwRoot) {
  CapRoot root{rwRoot};

  auto uart0 = uart_ptr(root);
  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);

  uart_tests(root, log);
  i2c_tests(root, log);
  spi_tests(root, log);
  hyperram_tests(root, log);
  usbdev_tests(root, log);
  pinmux_tests(root, log);
  plic_tests(root, log);
  finish_running(log, "All tests finished");
}
