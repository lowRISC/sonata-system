// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>
#include <ctype.h>

#include "sonata_system.h"
#include "i2c.h"
#include "dev_access.h"
#include "timer.h"
#include "rv_plic.h"

// Any Pi HAT with ID EEPROM present?
#define RPI_HAT_ID 1
// Raspberry Pi Sense HAT present?
#define RPI_HAT_SENSE 1
// AS6212 Temperature sensor present (Sparkfun)?
#define AS6212_TEMP_SENSOR 1


// A timeout of 1 second should be plenty; reading at 100kbps.
const uint32_t timeout_usecs = 1000 * 1000;

static const irq_t fmt_threshold_irq = 9;

void i2c_irq_test(irq_t irq) {
  puts("i2c fmt_threshold_irq received");
  rv_plic_disable(irq);
}

#if RPI_HAT_SENSE
// Read  "WHO_AM_I" registers in the IMU on the RPi Sense HAT
static int read_sense_imu(void) {
  uint8_t data[2];
  uint8_t addr[] = { 0x0f };

  i2c_t i2c = I2C_FROM_BASE_ADDR(I2C1_BASE);

  // Read from Accelerometer and gyroscope 'WHO_AM_I' register.
  if (i2c_write(i2c, 0x6a, addr, 1u, true) ||
    i2c_read(i2c, 0x6a, &data[0], 1u, timeout_usecs)) {
    puts("Failed to access Sense IMU");
    return -1;
  }
  putstr("Read 0x");
  puthex(data[0]);
  // Read from Magnetic Sensor 'WHO_AM_I' register.
  if (i2c_write(i2c, 0x1c, addr, 1u, true) ||
      i2c_read(i2c, 0x1c, &data[1], 1u, timeout_usecs)) {
    puts("Failed to access t'other one");
    return -2;
  }
  putstr(" - Read 0x");
  puthex(data[1]);
  if (data[0] != 0x68 || data[1] != 0x3d) {
    puts("*** READ FAILED ***");
    return -3;
  } else {
    puts(" -- PASS");
  }

  return 0;
}
#endif

#if AS6212_TEMP_SENSOR
// Report temperature from AS6212 temperature sensor (Sparkfun)
static int as6212_temperature_report(void) {
  uint8_t buf[2];

  i2c_t i2c = I2C_FROM_BASE_ADDR(I2C1_BASE);

  // Read from the Config register
  buf[0] = 1;
  if (i2c_write(i2c, 0x48, buf, 1u, false) ||
      i2c_read(i2c, 0x48, buf, 2u, timeout_usecs)) {
    return -1;
  }
  uint16_t config = (buf[0] << 8) | buf[1];
  // Read the Temperature
  buf[0] = 0;
  if (i2c_write(i2c, 0x48, buf, 1u, false) ||
      i2c_read(i2c, 0x48, buf, 2u, timeout_usecs)) {
    return -1;
  }
  uint16_t temp = (buf[0] << 8) | buf[1];

  putstr("AS612 Temperature Sensor - Config 0x");
  puthexn(config, 4);
  putstr(" Temp 0x");
  puthexn(temp, 4);
  putchar('\n');

  return 0;
}
#endif

#if RPI_HAT_ID
static int id_eeprom_report(i2c_t i2c) {
  const uint8_t kIdAddr = 0x50u;
  const uint8_t addr[] = { 0, 0 };
  // Data buffer is static to avoid placing a lot of data on the stack.
  static uint8_t data[0x80u];

  // Send two byte address (0x0000) and skip the STOP condition
  if (i2c_write(i2c, kIdAddr, addr, 2u, true)) {
    puts("Failed to set up address");
    return -1;
  }

  // Initialize the buffer to known contents in case of read issues.
  memset(data, 0xddu, sizeof(data));

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
#endif

static void setup_bus(i2c_t i2c) {
  // Select I2C host mode
  i2c_set_host_mode(i2c);
  // Program timing parameters
  i2c_set_speed(i2c, 100);
}

/**
 * Simple demonstration/test of I2C operation; use I2C0 to read the ID
 * EEPROM from a compliant Raspberry Pi HAT.
 */
int main(void) {
  uart_init(DEFAULT_UART);
  rv_plic_init();

  // This indicates how often the timer gets updated.
  timer_init();
  timer_enable(SYSCLK_FREQ / 15);

  // Two I2C controllers.
  i2c_t i2c0 = I2C_FROM_BASE_ADDR(I2C0_BASE);
  i2c_t i2c1 = I2C_FROM_BASE_ADDR(I2C1_BASE);

  // Test IRQ
  rv_plic_register_irq(fmt_threshold_irq, i2c_irq_test);
  rv_plic_enable(fmt_threshold_irq);
  // Set fmt threshold to 1
  DEV_WRITE(i2c0 + 0x24, 0x10000);
  // Enable fmt threshold IRQ
  DEV_WRITE(i2c0 + 0x4,  0x1);

  puts("i2c_hat_id demo application");

  setup_bus(i2c0);
  setup_bus(i2c1);

  uint64_t last_elapsed_time = get_elapsed_time();
  int iter = 0;
  while (1) {
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;

      int rc = 0;
#if RPI_HAT_ID
      // This displays a lot of output; do it less frequently.
      if (!(iter & 0x1f)) {
        rc = id_eeprom_report(i2c0);
        if (rc) {
          return rc;
        }
      }
#endif
#if RPI_HAT_SENSE
      rc = read_sense_imu();
      if (rc) {
        return rc;
      }
#endif
#if AS6212_TEMP_SENSOR
      rc = as6212_temperature_report();
      if (rc) {
        return rc;
      }
#endif
      iter++;
    }
  }
}

