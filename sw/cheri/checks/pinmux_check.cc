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
#include <cheri.hh>

#define LOG(...) write_str(uart, __VA_ARGS__)

using namespace CHERI;

#define PINMUX_SER0_TX_DISABLED (0)
#define PINMUX_SER0_TX_UART0_TX (1)

/**
 * Blocks until the UART transmit FIFO is empty.
 */
void block_until_uart_tx_done(Capability<volatile OpenTitanUart> uart) {
  while (uart->transmit_fifo_level() > 0) {
    asm volatile("");
  }
}

/**
 * Simple pinmux check. Logs a message to the default UART0 console.
 * Then disables UART0's connection to Serial0Transmit via pinmux.
 * Then logs again. Then re-enables the connection, and logs one more time.
 * If pinmux is working, then you should not see the middle message.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;
  uart->init(BAUD_RATE);

  Capability<volatile uint8_t> pinmux = root.cast<volatile uint8_t>();
  pinmux.address()                    = PINMUX_ADDRESS;
  pinmux.bounds()                     = PINMUX_BOUNDS;

  SonataPinmux Pinmux = SonataPinmux(pinmux);

  LOG("Starting check.\r\n");

  LOG("You should see this message, as UART0 is pinmuxed by default.\r\n");
  block_until_uart_tx_done(uart);

  // Pinmux Serial 0 TX (used by console UART) to OFF.
  Pinmux.output_pin_select(SonataPinmux::OutputPin::Serial0Transmit, PINMUX_SER0_TX_DISABLED);
  LOG("You should NOT see this message, as we just used pinmux to disable UART0 output.\r\n");
  block_until_uart_tx_done(uart);

  // Pinmux Serial 0 TX (used by console UART) to UART0_TX.
  Pinmux.output_pin_select(SonataPinmux::OutputPin::Serial0Transmit, PINMUX_SER0_TX_UART0_TX);
  LOG("You should see this message, as UART0 has just been re-enabled.\r\n");

  LOG("Check completed.\r\n");

  while (true) {
    asm volatile("");
  }
}
