// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "test_runner.hh"

// clang-format off
#include "../../common/defs.h"
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
#include "spi_tests.hh"
#include "uart_tests.hh"

extern "C" void entry_point(void *rwRoot) {
  CapRoot root{rwRoot};

  auto console = uart_ptr(root);

  console->init(BAUD_RATE);
  uart_tests(root, console);
  spi_tests(root, console);
  hyperram_tests(root, console);
  plic_tests(root, console);
  finish_running(console, "All tests finished");
}
