// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>

#include "i2c_as621x.hh"

// Bus reset; reset the device state.
void i2c_as621x::busReset() {
  i2cdevice::busReset();
  index = 0u;
  byteCount = 0u;
}

// Start condition occurred on the I2C bus (broadcast to all devices).
void i2c_as621x::startCond() {
  i2cdevice::startCond();
  byteCount = 0u;
}

// Write a byte of data to the AS612x Digital Temperature Sensor.
bool i2c_as621x::writeByte(uint8_t inByte, uint32_t oobIn) {
  // Word write programs the Index register and then optionally the 16 bits of the register
  // selected by the Index register.
  switch (byteCount) {
    case 0u:
      if (!(index >> 2)) index = inByte & 3u;
      break;
    case 1u: dataHi = inByte; break;
    case 2u:  {
      uint16_t val = ((uint16_t)dataHi << 8) | inByte;
      switch (index) {
        case 0u: break; // 'tval' register is Read Only
        case 1u: config = val; break;
        case 2u: tlow   = val; break;
        default: thigh  = val; break;
      }
    }
    break;
  }
  byteCount++;
  // Byte accepted (send ACK).
  return true;
}

// Read a byte of data from the AS612x Digital Temperature Sensor.
bool i2c_as621x::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  // Collect the contents of the selected register; Word Read returns 16 bits from the register
  // indicated by the Index register which must have been written already.
  uint16_t val;
  switch (index & 3u) {
    case 0u: {
      // Random fluctuations of temperature across the range 24-26 degrees.
      val = 0xc00u + (rand() & 0xffu);
    }
    break;
    case 1u: val = config; break;
    case 2u: val = tlow;   break;
    default: val = thigh;  break;
  }

  logText("AS621x returning 0x%04x from reg %u\n", val, index & 3u);
  outByte = (byteCount & 1u) ? (uint8_t)val : (uint8_t)(val >> 8);
  byteCount ^= 1u;
  // Byte available, transmit to controller.
  return true;
}
