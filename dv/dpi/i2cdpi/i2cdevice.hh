// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DV_DPI_I2CDPI_I2CDEVICE_H_
#define __DV_DPI_I2CDPI_I2CDEVICE_H_
#include <stdint.h>

// I2C addresses are actually 7 bits typically.
typedef uint8_t i2caddr_t;

class i2cdevice {
public:
  i2cdevice(i2caddr_t addr, bool log = false) : logging(log), devAddress(addr) {
    busReset();
  }
  virtual ~i2cdevice() { }

  // Return the I2C address of this device.
  i2caddr_t getAddress() const { return devAddress; }

  // Bus reset.
  virtual void busReset(void) { }

  // Start condition occurred on the I2C bus (broadcast to all devices).
  virtual void startCond() { }

  // Restart condition occurred on the I2C bus for this specific device.
  virtual void restartCond() { }

  // Access starting.
  virtual void accessStarting(bool read) { }

  // Stop condition occurred on the I2C bus for this specific device.
  virtual void stopCond() { }

  // Write a byte of data to the I2C device.
  virtual bool writeByte(uint8_t inByte, uint32_t oobIn) {
    // Sink consumes all write traffic.
    return true;
  }

  // Read a byte of data from the I2C device.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
    // Sources no read traffic.
    outByte = 0xffu; // Open drain bus.
    oobOut = 0u;
    return false;
  }

protected:
  // Diagnostic logging output.
  void logText(const char *fmt, ...);

  // Enable diagnostic logging output?
  bool logging;

  i2caddr_t devAddress;
};
#endif
