// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>
#include <platform-uart.hh>

#include "../common/uart-utils.hh"

extern "C" uint32_t entry_point(void *rwRoot) {
  using namespace CHERI;
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart<>> uart = root.cast<volatile OpenTitanUart<>>();
  uart.address()                            = UART_ADDRESS;
  uart.bounds()                             = UART_BOUNDS;

  uart->init(BAUD_RATE);
  write_str(uart, "Got second flash read:\r\n");

  return 0;
}
