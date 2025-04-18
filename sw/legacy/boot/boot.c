// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>

#include "gpio.h"
#include "pwm.h"
#include "sonata_system.h"
#include "timer.h"

/**
 * Is the code running in simulation (as opposed to on FPGA)?
 */
#define SIMULATION 0

static const bool dual_uart = true;
static uart_t uart0, uart1;

static inline void write_both(char ch0, char ch1) {
  uart_out(uart0, ch0);
  uart_out(uart1, ch1);
}

int main(void) {
  if (dual_uart) {
    const char *signon = "Legacy boot code; UART ";
    uart0              = UART_FROM_BASE_ADDR(UART0_BASE);
    uart1              = UART_FROM_BASE_ADDR(UART1_BASE);
    uart_init(uart0);
    uart_init(uart1);
    // Send a sign-on message to both UARTs.
    char ch;
    while ('\0' != (ch = *signon)) {
      write_both(ch, ch);
      signon++;
    }
    write_both('0', '1');
    write_both('\r', '\r');
    write_both('\n', '\n');
  } else {
    uart0 = DEFAULT_UART;
    uart_init(uart0);
    puts("Legacy boot code");
  }

  // This indicates how often the timer gets updated.
  timer_init();
#if SIMULATION
  timer_enable(SYSCLK_FREQ / 500);
#else
  timer_enable(SYSCLK_FREQ / 5);
#endif

  uint64_t last_elapsed_time = get_elapsed_time();

  // Reset user LEDs to having just one on
  set_outputs(GPIO_OUT, 0x01);

  while (1) {
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;

      // Cycle through LEDs
      uint32_t out_val = read_gpio(GPIO_OUT);
      out_val          = (out_val << 1) & GPIO_LED_MASK;
      out_val |= !out_val;
      set_outputs(GPIO_OUT, out_val);
    }

    asm volatile("wfi");
  }
}
