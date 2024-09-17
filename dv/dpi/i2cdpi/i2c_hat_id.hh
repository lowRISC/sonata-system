// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DV_DPI_I2CDPI_I2C_HAT_ID_H_
#define __DV_DPI_I2CDPI_I2C_HAT_ID_H_
#include "i2cdevice.hh"

// Model of ID EEPROM on the Raspberry Pi Sense HAT.
class i2c_hat_id : public i2cdevice {
public:
  i2c_hat_id(i2caddr_t addr);

  // Physical properties of the EEPROM.
  static constexpr unsigned kPageSize = 0x20u;   // Pages are 32 bytes.
  static constexpr unsigned kMemSize  = 0x1000u; // 32Kib.
protected:
  // Bus reset.
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

  // Current address within the EEPROM.
  uint16_t currAddr;

  // Memory contents.
  uint8_t mem[kMemSize];
};
#endif  // __DV_DPI_I2CDPI_I2C_HAT_ID_H_
