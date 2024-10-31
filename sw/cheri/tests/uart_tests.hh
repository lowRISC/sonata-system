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

/**
 * Configures the number of test iterations to perform.
 * This can be overriden via a compilation flag.
 */
#ifndef UART_TEST_ITERATIONS
#define UART_TEST_ITERATIONS (1U)
#endif

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

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < UART_TEST_ITERATIONS; i++) {
    log.println("\r\nrunning uart_test: {} \\ {}", i, UART_TEST_ITERATIONS - 1);
    bool test_failed = false;
    int failures     = 0;

    log.print("  Running UART Loopback test... ");
    failures = !uart_loopback_test(uart1);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running UART Interrupt State test... ");
    failures = !uart_interrupt_state_test(uart1);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    check_result(log, !test_failed);
  }
}
