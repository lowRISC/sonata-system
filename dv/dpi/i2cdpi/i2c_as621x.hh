// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "i2cdevice.hh"

// Model of AS621x Digital Temperature Sensor.
class i2c_as621x : public i2cdevice {
public:
  i2c_as621x(i2caddr_t addr) : i2cdevice(addr) {
    busReset();
  }

protected:
  // Bus reset condition.
  virtual void busReset();

  // Start condition occurred on the I2C bus (broadcast to all devices).
  virtual void startCond();

  // Write a byte of data to the AS621x Digital Temperature Sensor.
  virtual bool writeByte(uint8_t inByte, uint32_t oobIn);

  // Read a byte of data from the AS621x Digital Temperature Sensor.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut);

private:
  // Number of bytes transferred in current transcation.
  uint8_t byteCount;

  // High byte of two-byte write operation (the MS byte is transmitted first).
  uint8_t dataHi;

  // Current temperature value (tval) is not stored; value is generated when read.
  // CONFIGuration register.
  uint16_t config;
  // TLOW and THIGH temperature threshold values.
  uint16_t tlow;
  uint16_t thigh;
  // Index register selects amongt the 4 registers above.
  uint8_t  index;
};
