/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include "../common/uart-utils.hh"
#include "../common/platform-pinmux.hh"
#include "../common/sonata-devices.hh"
#include <cheri.hh>

#define LOG(...) write_str(uart, __VA_ARGS__)

using namespace CHERI;

/**
 * Blocks until the UART transmit FIFO is empty.
 */
void block_until_uart_tx_done(Capability<volatile OpenTitanUart> uart) {
  while (!(uart->status & uart->StatusTransmitIdle));
}

/**
 * Simple pinmux check. Logs a message to the default UART0 console.
 * Then disables UART0's connection to Serial0Transmit via pinmux.
 * Then logs again. Then re-enables the connection, and logs one more time.
 * If pinmux is working, then you should not see the middle message.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  UartPtr uart = uart_ptr(root);
  uart->init(BAUD_RATE);

  SonataPinmux::Sink ser0_tx = pin_sinks_ptr(root)->get(SonataPinmux::PinSink::ser0_tx);

  LOG("Starting check.\r\n");

  LOG("You should see this message, as UART0 is pinmuxed by default.\r\n");
  block_until_uart_tx_done(uart);

  // Pinmux Serial 0 TX (used by console UART) to OFF.

  ser0_tx.disable();
  LOG("You should NOT see this message, as we just used pinmux to disable UART0 output.\r\n");
  block_until_uart_tx_done(uart);

  // Pinmux Serial 0 TX (used by console UART) to UART0_TX.
  ser0_tx.default_selection();
  LOG("You should see this message, as UART0 has just been re-enabled.\r\n");

  LOG("Check completed.\r\n");

  while (true) {
    asm volatile("");
  }
}
