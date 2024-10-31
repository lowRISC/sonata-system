// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "i2c.h"

#include "dev_access.h"
#include "sonata_system.h"

/**
 * Specification times (Table 10) in nanoseconds for each bus mode.
 */
static const uint16_t spc_thigh[]   = {4000u, 600u, 260u};
static const uint16_t spc_tlow[]    = {4700u, 1300u, 150u};
static const uint16_t spc_thd_sta[] = {4000u, 600u, 260u};
static const uint16_t spc_tsu_sta[] = {4700u, 600u, 260u};
static const uint16_t spc_thd_dat[] = {5000u, 1u, 1u};
static const uint16_t spc_tsu_dat[] = {250u, 100u, 50u};
static const uint16_t spc_t_buf[]   = {4700u, 1300u, 500u};
static const uint16_t spc_tsu_sto[] = {4000u, 600u, 260u};

/**
 * Performs a 32-bit integer unsigned division, rounding up. The bottom
 * 16 bits of the result are then returned.
 *
 * As usual, a divisor of 0 is still Undefined Behavior.
 */
static uint16_t round_up_divide(uint32_t a, uint32_t b) {
  const uint32_t result = ((a - 1) / b) + 1;
  return (uint16_t)result;
}

static int write_raw_fmt(i2c_t i2c, uint32_t fmt) {
  while (0u != (I2C_STATUS_FMTFULL & DEV_READ(i2c + I2C_STATUS))) {
    // FMT FIFO should drain quickly as the host transmits
  }
  DEV_WRITE(i2c + I2C_FDATA, fmt);
  return 0;
}

static int write_raw_bytes(i2c_t i2c, const uint8_t *data, size_t n) {
  while (n-- > 0u) {
    while (0u != (I2C_STATUS_FMTFULL & DEV_READ(i2c + I2C_STATUS))) {
      // FMT FIFO should drain quickly as the host transmits
    }
    DEV_WRITE(i2c + I2C_FDATA, *data);
    data++;
  }
  return 0;
}

static int wait_fmt_empty(i2c_t i2c) {
  while (0 == (I2C_STATUS_FMTEMPTY & DEV_READ(i2c + I2C_STATUS))) {
    // FMT FIFO should empty quickly
  }
  return 0;
}

void i2c_set_host_mode(i2c_t i2c) { DEV_WRITE(i2c + I2C_CTRL, I2C_CTRL_ENABLEHOST); }

/**
 * Set the I2C timing parameters appropriately for the given bit rate.
 * Distilled from:
 * https://opentitan.org/book/hw/ip/i2c/doc/programmers_guide.html
 */
void i2c_set_speed(i2c_t i2c, unsigned speed_khz) {
  // We must round up the system clock frequency to lengthen intervals.
  const uint32_t sysclk_khz = (SYSCLK_FREQ + 999) / 1000;
  // We want to underestimate the clock period, to lengthen the timings.
  uint32_t clk_period = (1000 * 1000) / sysclk_khz;

  // Decide which bus mode this represents
  int mode = (speed_khz > 100u) + (speed_khz > 400u);

  // Calculation of timing parameters
  uint16_t thigh = round_up_divide(spc_thigh[mode], clk_period);  // Spec. min.
  uint16_t tlow  = round_up_divide(spc_tlow[mode], clk_period);   // Spec. min.
  uint16_t t_f   = round_up_divide(20 * 3 / 5, clk_period);       // Spec. min. 3.3V
  uint16_t t_r   = round_up_divide(120, clk_period);
  // Setup and Hold times for Start
  uint16_t thd_sta = round_up_divide(spc_thd_sta[mode], clk_period);
  uint16_t tsu_sta = round_up_divide(spc_tsu_sta[mode], clk_period);
  // Setup and Hold times for Data
  uint16_t thd_dat = round_up_divide(spc_thd_dat[mode], clk_period);
  uint16_t tsu_dat = round_up_divide(spc_tsu_dat[mode], clk_period);
  uint16_t t_buf   = round_up_divide(spc_t_buf[mode], clk_period);
  uint16_t tsu_sto = round_up_divide(spc_tsu_sto[mode], clk_period);

  // Prevent counters underflowing
  if (tlow < thd_dat + 1u) tlow = thd_dat + 1u;
  if (t_buf < tsu_sta + 1u) t_buf = tsu_sta + 1u;

  if (true) {
    putstr("clk_period: ");
    putdec(clk_period);
    putstr("\nthigh:   ");
    putdec(thigh);
    putstr("\ntlow:    ");
    putdec(tlow);
    putstr("\nt_f:     ");
    putdec(t_f);
    putstr("\nt_r:     ");
    putdec(t_r);
    putstr("\nthd_sta: ");
    putdec(thd_sta);
    putstr("\ntsu_sta: ");
    putdec(tsu_sta);
    putstr("\nthd_dat: ");
    putdec(thd_dat);
    putstr("\ntsu_dat: ");
    putdec(tsu_dat);
    putstr("\nt_buf:   ");
    putdec(t_buf);
    putstr("\ntsu_sto: ");
    putdec(tsu_sto);
    putchar('\n');
  }

  DEV_WRITE(i2c + I2C_TIMING0, (tlow << 16) | thigh);
  DEV_WRITE(i2c + I2C_TIMING1, (t_f << 16) | t_r);
  DEV_WRITE(i2c + I2C_TIMING2, (thd_sta << 16) | tsu_sta);
  DEV_WRITE(i2c + I2C_TIMING3, (thd_dat << 16) | tsu_dat);
  DEV_WRITE(i2c + I2C_TIMING4, (t_buf << 16) | tsu_sto);
}

void reset_controller_events(i2c_t i2c) {
  const uint32_t field_mask = (I2C_CONTROLLER_EVENTS_NACK | I2C_CONTROLLER_EVENTS_UNHANDLED_NACK_TIMEOUT |
                               I2C_CONTROLLER_EVENTS_BUS_TIMEOUT | I2C_CONTROLLER_EVENTS_ARBITRATION_LOST);
  DEV_WRITE(i2c + I2C_CONTROLLER_EVENTS, field_mask);
}

int i2c_write(i2c_t i2c, uint8_t addr7, const uint8_t *data, size_t n, bool skip_stop) {
  // Set up a write to the given target address.
  int rc = write_raw_fmt(i2c, I2C_FDATA_START | (addr7 << 1) | 0u);
  if (rc) {
    return rc;
  }
  // Send the write data, deferring the final byte.
  if (n > 1) {
    write_raw_bytes(i2c, data, n - 1);
  }
  rc = write_raw_fmt(i2c, (skip_stop ? 0u : I2C_FDATA_STOP) | data[n - 1]);
  // While format is not empty, check for controller halts and fail if one occurs.
  while (0 != (I2C_STATUS_FMTEMPTY & DEV_READ(i2c + I2C_STATUS))) {
    if (I2C_INTR_STATE_CONTROLLER_HALT & DEV_READ(i2c + I2C_INTR_STATE)) {
      // Clear the Controller Halt (NAK) and return failure.
      reset_controller_events(i2c);
      rc = -1;
      break;
    }
  }
  return rc;
}

int i2c_read(i2c_t i2c, uint8_t addr7, uint8_t *buf, size_t n, uint32_t timeout_usecs) {
  uint32_t timeout_cycles = (uint32_t)((uint64_t)timeout_usecs * (SYSCLK_FREQ / 1000) / 1000);
  uint32_t start_time     = get_mcycle();
  // The I2C controller may read only a limited number of bytes at a time.
  int rc = 0u;
  while (n > 0u) {
    // Set up a read from the given target address.
    rc = write_raw_fmt(i2c, I2C_FDATA_START | (addr7 << 1) | 1u);
    if (!rc) {
      rc = wait_fmt_empty(i2c);
      if (!rc) {
        if (I2C_INTR_STATE_CONTROLLER_HALT & DEV_READ(i2c + I2C_INTR_STATE)) {
          // Clear the Controller Halt (NAK) and return failure.
          reset_controller_events(i2c);
          rc = -1;
        }
        // else no NAK interrupt, so ACK received.
      }
    }
    if (rc) {
      break;
    }

    // Instruct the I2C controller how many bytes to read.
    uint8_t chunk = (n >= UINT8_MAX) ? UINT8_MAX : (uint8_t)n;
    write_raw_fmt(i2c, ((chunk >= n) ? I2C_FDATA_STOP : 0u) | I2C_FDATA_READB | chunk);
    n -= chunk;

    // Wait until the read data is available, or NAK.
    rc = wait_fmt_empty(i2c);
    if (!rc) {
      while (chunk-- > 0u) {
        if (get_mcycle() - start_time >= timeout_cycles) {
          // Timeout elapsed.
          rc = -1;
          break;
        }
        *buf++ = DEV_READ(i2c + I2C_RDATA);
      }
    }
  }

  return rc;
}
