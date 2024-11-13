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
#include "../common/timer-utils.hh"
#include "../common/sonata-devices.hh"
#include <cheri.hh>

#define RS485_UART 2

using namespace CHERI;

[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  pin_sinks_ptr(root)->get(SonataPinmux::PinSink::rs485_tx).select(1);
  block_sinks_ptr(root)->get(SonataPinmux::BlockSink::uart_2_rx).select(3);

  UartPtr uart = uart_ptr(root, 0);
  uart->init(BAUD_RATE);

  Capability<volatile OpenTitanUart> rs485_uart = root.cast<volatile OpenTitanUart>();
  rs485_uart.address()                          = UART_ADDRESS + RS485_UART * UART_RANGE;
  rs485_uart.bounds()                           = UART_BOUNDS;
  rs485_uart->init(BAUD_RATE);

  write_str(uart, "Writing to RS-485 Uart\r\n");

  while (1) {
    write_str(rs485_uart, "Hello from RS-485! Type something and terminate with enter (\\r):\r\n");

    char uart_capture_buf[256];
    uint32_t chars_seen = 0;
    char ch;

    do {
      ch = rs485_uart->blocking_read();

      write_str(uart, "Saw RS-485: 0x");
      write_hex8b(uart, ch);
      write_str(uart, "\r\n");

      // Truncate after 255 captured characters to keep room for null terminator
      if (chars_seen < 255) {
        uart_capture_buf[chars_seen++] = ch;
      }
    } while (ch != '\r');

    // Strip off '\r' and zero terminate
    uart_capture_buf[chars_seen - 1] = 0;

    write_str(uart, "Received from RS-485: ");
    write_str(uart, uart_capture_buf);
    write_str(uart, "\r\n");

    write_str(rs485_uart, "Received: ");
    write_str(rs485_uart, uart_capture_buf);
    write_str(rs485_uart, "\r\n");
  }
}
