// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "uart.h"

#include "sonata_system.h"
#include "dev_access.h"

#define BAUD_RATE (921600)

void uart_enable_rx_int(void) {
  enable_interrupts(UART_IRQ);
  arch_local_irq_enable();
}

int uart_init(uart_t uart) {
  // NCO = 2^20 * baud rate / cpu frequency
  uint32_t nco = (uint32_t)(((uint64_t)BAUD_RATE << 20) / SYSCLK_FREQ);

  DEV_WRITE(uart + UART_CTRL_REG, (nco << 16) | 0x3U);

  return 0;
}

int uart_in(uart_t uart) {
  int res = UART_EOF;

  if (!(DEV_READ(uart + UART_STATUS_REG) & UART_STATUS_RX_EMPTY)) {
    res = DEV_READ(uart + UART_RX_REG);
  }

  return res;
}

void uart_out(uart_t uart, char c) {
  while (DEV_READ(uart + UART_STATUS_REG) & UART_STATUS_TX_FULL)
    ;

  DEV_WRITE(uart + UART_TX_REG, c);
}
