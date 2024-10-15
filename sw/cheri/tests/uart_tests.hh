// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>

// clang-format off
#include "../../common/defs.h"
#include <cheri.hh>
// clang-format on
#include <platform-uart.hh>

#include "../common/sonata-devices.hh"
#include "../common/uart-utils.hh"
#include "test_runner.hh"

const char UartLoopbackTestString[] = "test string";

bool uart_loopback_test(UartPtr uart) {
  uart->init();
  uart->fifos_clear();
  uart->parity();
  uart->loopback();

  for (char c : UartLoopbackTestString) {
    uart->blocking_write(c);
  }
  for (char c : UartLoopbackTestString) {
    if (uart->blocking_read() != c) {
      return false;
    }
  }
  return true;
}

bool uart_interrupt_state_test(UartPtr uart) {
  uart->init();
  uart->fifos_clear();
  uart->parity();
  uart->transmit_watermark(OpenTitanUart::TransmitWatermark::Level4);

  uint32_t count = 0;
  while (uart->interruptState & OpenTitanUart::InterruptTransmitWatermark) {
    uart->blocking_write('x');
    ++count;
  }

  return count == 5;
}

void uart_tests(CapRoot root, Log& log) {
  auto uart1 = uart_ptr(root, 1);

  log.println("running uart_loopback_test");
  check_result(log, uart_loopback_test(uart1));
  log.println("running uart_interrupt_state_test");
  check_result(log, uart_interrupt_state_test(uart1));
}
