// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DV_DPI_I2CDPI_I2C_LSM9DS1_H_
#define __DV_DPI_I2CDPI_I2C_LSM9DS1_H_
#include "i2cdevice.hh"

// iNEMO intertial module: 3D accelerometer, 3D gyroscope, 3D magnetometer.
// Included on the Raspberry Pi Sense HAT. Model just reponds to reads from 'WHO_AM_I' ID registers.
class i2c_lsm9ds1 : public i2cdevice {
public:
  // Note: this IMU device is unusual in that it presents as two devices, at distinct addresses
  // on the I2C bus. Therefore two device instances are required to model the full functionality.
  i2c_lsm9ds1(i2caddr_t addr) : i2cdevice(addr) {
    busReset();
  }

protected:
  // Bus reset.
  virtual void busReset();

  // Write a byte of data to the LSM9DS1 IMU.
  virtual bool writeByte(uint8_t inByte, uint32_t oobIn);

  // Read a byte of data from the LSM9DS1 IMU.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut);
private:
  // Selected register.
  uint8_t regNum;
};
#endif  // __DV_DPI_I2CDPI_I2C_LSM9DS1_H_
