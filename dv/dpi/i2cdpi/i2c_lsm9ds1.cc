// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "i2c_lsm9ds1.hh"

// Device reset.
void i2c_lsm9ds1::busReset() {
  i2cdevice::busReset();
  regNum = 0u;
}

// Write a byte of data to the LSM9DS1 IMU.
bool i2c_lsm9ds1::writeByte(uint8_t inByte, uint32_t oobIn) {
  // The model is very limited; we are interested only in the 'WHO_AM_I' ID registers for now.
  regNum = inByte;
  // Byte accepted, send ACK.
  return true;
}

// Read a byte of data from LSM9DS1 IMU.
bool i2c_lsm9ds1::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  // The model is very limited; we are interested only in the 'WHO_AM_I' ID registers for now.
  switch (regNum) {
    case 0x0fu: {
      // This single IC presents as two I2C targets.
      switch (devAddress) {
        // Accelerometer and Gyroscope.
        case 0x6au: outByte = 0x68u; break;
        // Magnetic sensor.
        case 0x1cu: outByte = 0x3du; break;
        default: outByte = 0xffu; break;
      }
    }
    break;
    default: outByte = 0xffu; break;
  }
  oobOut = 0u;
  // Byte available, transmit to controller.
  return true;
}
