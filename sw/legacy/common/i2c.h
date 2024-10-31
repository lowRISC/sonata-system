// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef I2C_H__
#define I2C_H__
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Register definitions for the relevant parts of the OpenTitan I2C block.
 * Distilled from https://opentitan.org/book/hw/ip/i2c/doc/registers.html
 */

// Byte offsets of key registers
#define I2C_INTR_STATE 0x00
#define I2C_INTR_ENABLE 0x04
#define I2C_CTRL 0x10
#define I2C_STATUS 0x14
#define I2C_RDATA 0x18
#define I2C_FDATA 0x1C
#define I2C_TIMING0 0x3C
#define I2C_TIMING1 0x40
#define I2C_TIMING2 0x44
#define I2C_TIMING3 0x48
#define I2C_TIMING4 0x4C
#define I2C_CONTROLLER_EVENTS 0x78

// I2C_INTR_STATE register
#define I2C_INTR_STATE_CONTROLLER_HALT (1 << 4)

// I2C_CTRL register
#define I2C_CTRL_ENABLEHOST 1

// I2C STATUS register
#define I2C_STATUS_FMTFULL 1
#define I2C_STATUS_FMTEMPTY 4
#define I2C_STATUS_RXEMPTY 0x20

// I2C FDATA register
#define I2C_FDATA_READB 0x400
#define I2C_FDATA_STOP 0x200
#define I2C_FDATA_START 0x100

// I2C ControllerEvents register
#define I2C_CONTROLLER_EVENTS_NACK (1u << 0)
#define I2C_CONTROLLER_EVENTS_UNHANDLED_NACK_TIMEOUT (1u << 1)
#define I2C_CONTROLLER_EVENTS_BUS_TIMEOUT (1u << 2)
#define I2C_CONTROLLER_EVENTS_ARBITRATION_LOST (1 << 3)

// Should be treated as opaque
typedef uintptr_t i2c_t;

#define I2C_FROM_BASE_ADDR(addr) ((i2c_t)(addr))

void i2c_set_host_mode(i2c_t i2c);
void i2c_set_speed(i2c_t i2c, unsigned speed_khz);
int i2c_write(i2c_t i2c, uint8_t addr7, const uint8_t *data, size_t n, bool skip_stop);
int i2c_read(i2c_t i2c, uint8_t addr7, uint8_t *buf, size_t n, uint32_t timeout_usecs);

#endif
