// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>
#include <ctype.h>

#include "sonata_system.h"
#include "i2c.h"
#include "rv_plic.h"
#include "dev_access.h"

// A timeout of 1 second should be plenty; reading at 100kbps.
const uint32_t timeout_usecs = 1000 * 1000;

static const irq_t fmt_threshold_irq = 9;

void i2c_irq_test(irq_t irq) {
  puts("i2c fmt_threshold_irq received");
  rv_plic_disable(irq);
}

/**
 * Simple demonstration/test of I2C operation; use I2C0 to read the ID
 * EEPROM from a compliant Raspberry Pi HAT.
 */
int main(void) {
  const uint8_t kIdAddr = 0x50u;
  const uint8_t addr[] = { 0, 0 };
  // Data buffer is static to avoid placing a lot of data on the stack.
  static uint8_t data[0x80u];

  uart_init(DEFAULT_UART);
  rv_plic_init();

  // Test IRQ
  rv_plic_register_irq(fmt_threshold_irq, i2c_irq_test);
  rv_plic_enable(fmt_threshold_irq);
  // Set fmt threshold to 1
  DEV_WRITE(DEFAULT_I2C + 0x24, 0x10000);
  // Enable fmt threshold IRQ
  DEV_WRITE(DEFAULT_I2C + 0x4,  0x1);

  puts("i2c_hat_id demo application");

  i2c_t i2c = DEFAULT_I2C;

  // Select I2C host mode
  i2c_set_host_mode(i2c);
  // Program timing parameters
  i2c_set_speed(i2c, 100);

  // Send two byte address (0x0000) and skip the STOP condition
  if (i2c_write(i2c, kIdAddr, addr, 2u, true)) {
    puts("Failed to set up address");
    return -1;
  }

  // Initialize the buffer to known contents in case of read issues.
  memset(data, 0xffu, sizeof(data));

  // Read a 256 byte page from the ID EEPROM
  if (i2c_read(i2c, kIdAddr, data, sizeof(data), timeout_usecs)) {
    puts("Failed to read EEPROM ID contents from Pi HAT");
    return -2;
  }

  unsigned idx = 0u;
  while (idx < sizeof(data)) {
    unsigned eidx = idx + 0x10u;
    unsigned i;
    // Offset within page
    puthex(idx);
    putchar(' ');
    putchar(':');
    putchar(' ');
    // Each byte as hexadecimal
    for (i = idx; i < eidx; i++) {
      puthexn(data[i], 2u);
      putchar(' ');
    }
    putchar(':');
    putchar(' ');
    // Each byte as printable character
    for (i = idx; i < eidx; i++) {
      char ch = (char)data[i];
      ch = isprint(ch) ? ch : '.';
      putchar(ch);
    }
    putchar('\n');
    idx = eidx;
  }

  return 0;
}

