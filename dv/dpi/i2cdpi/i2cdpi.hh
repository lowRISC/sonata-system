// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DV_DPI_I2CDPI_I2CDPI_H_
#define __DV_DPI_I2CDPI_I2CDPI_H_
#include <map>
#include "i2cdevice.hh"

// DPI model of an I2C bus and the communication with attached devices; the DPI model performs all
// the work of serialising/deserialising the byte-level traffic, address decoding and the handling
// of ACK/NAK signalling.
//
// Communication with the attached I2C device models occurs at the level of bytes to keep them
// simple. Devices are also informed of (S)tart, repeated start (Sr) and sto(P) conditions if they
// require.
class i2cdpi {
public:
  i2cdpi(bool log = false) :
    logging(log),
    prev_rst(false),
    prev_scl(true),
    prev_sda(true),
    latestOobOut(0u),
    currByte(0u),
    numBits(0u),
    currDevice(nullptr),
    reading(false),
    sendAck(true) { }

  // Destructor for I2C DPI model; deletes all attached devices.
  ~i2cdpi() {
    for (const auto &dev : devs) dev.second->startCond();
    devs.clear();
  }

  // Add a device to this bus.
  void add_device(i2cdevice *dev) {
    (void)devs.insert(std::pair<i2caddr_t, i2cdevice *>(dev->getAddress(), dev));
  }

  // Return access to the device at the given target address, iff present.
  i2cdevice *find_device(i2caddr_t addr) {
    auto match = devs.find(addr);
    return (match == devs.end()) ? nullptr : (*match).second;
  }

  // Bus reset signalling.
  uint32_t under_reset(bool scl, bool sda, uint32_t oobIn, uint32_t &oobOut);

  // Decode I2C bus signalling.
  uint32_t decode(bool scl, bool sda, uint32_t oobIn, uint32_t &oobOut);

  // Diagnostic logging utility function.
  void logText(const char *fmt, ...);

  // Enable diagnostic logging output?
  bool logging;

private:
  // Broadcast(S)tart condition to all devices; we cannot yet know which is being addressed.
  inline void start_cond() {
    for (const auto &dev : devs) dev.second->startCond();
  }

  // Current state of I2C bus/decoding.
  enum {
    I2C_Idle,
    I2C_Start,
    I2C_Restart,
    I2C_Addr,
    I2C_GotAddr,
    I2C_SendAckNak,
    I2C_Data,
    I2C_GotData,
    I2C_SentData,
    I2C_GotAckNak
  } state;

  // Previous state of Bus Reset signal.
  bool prev_rst;
  // Previous state of I2C bus.
  bool prev_scl;
  bool prev_sda;

  // Most recent Out-Of-Band output signals on this I2C bus.
  uint32_t latestOobOut;

  // Current input/output byte.
  uint8_t currByte;
  // Number of bits received/transmitted (within current byte).
  uint8_t numBits;

  // Target device for the current transaction.
  i2cdevice *currDevice;

  // Current transaction is a read operation.
  bool reading;

  // Device has chosen to ACK the current byte of a write transaction.
  bool sendAck;

  // Devices connected to this bus.
  typedef std::pair<i2caddr_t, i2cdevice *> dev_entry;
  std::map<i2caddr_t, i2cdevice *> devs;
};
#endif  // __DV_DPI_I2CDPI_I2CDPI_H_
