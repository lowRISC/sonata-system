// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_hat_id.hh"

// Captured EEPROM ID header from Raspberry Pi Sense HAT; this device is not Read Only and it
// supports Byte/Page Writes too. This static signature is used to initialise the memory.
static const uint8_t id[] = {
 0x52, 0x2D, 0x50, 0x69, 0x01, 0x00, 0x03, 0x00,
 0xF0, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, // R-Pi............
 0x2D, 0x00, 0x00, 0x00, 0xE7, 0x58, 0xD1, 0x95,
 0xF1, 0x56, 0x28, 0xAB, 0xCB, 0x4E, 0x99, 0x42, // -....X...V(..N.B
 0xC7, 0x79, 0xD6, 0xA3, 0x01, 0x00, 0x01, 0x00,
 0x0C, 0x09, 0x52, 0x61, 0x73, 0x70, 0x62, 0x65, // .y........Raspbe
 0x72, 0x72, 0x79, 0x20, 0x50, 0x69, 0x53, 0x65,
 0x6E, 0x73, 0x65, 0x20, 0x48, 0x41, 0x54, 0x7F, // rry PiSense HAT.
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ................
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ................
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ................
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // ................
};

// Both of these are physical device properties so they should not be changed anyway,
// but they must be powers of two for the logic to work.
static_assert(!(i2c_hat_id::kMemSize  & (i2c_hat_id::kMemSize  - 1u)));
static_assert(!(i2c_hat_id::kPageSize & (i2c_hat_id::kPageSize - 1u)));

// Constructor initialises the device state.
i2c_hat_id::i2c_hat_id(i2caddr_t addr) : i2cdevice(addr) {
  busReset();
  // Initialise the memory contents from the static signature above.
  assert(sizeof(mem) >= sizeof(id));
  memcpy(mem, id, sizeof(id));
}

// Bus reset condition; reset the device.
void i2c_hat_id::busReset() {
  i2cdevice::busReset();
  byteCount = 0u;
  currAddr = 0u;
}

// Start condition occurred on the I2C bus (broadcast to all devices).
void i2c_hat_id::startCond() {
  i2cdevice::startCond();
  byteCount = 0u;
}

// Write a byte of data to the the RPi Sense HAT ID EEPROM.
bool i2c_hat_id::writeByte(uint8_t inByte, uint32_t oobIn) {
  switch (byteCount) {
    case 0u: currAddr = (uint16_t)inByte << 8; break;
    case 1u: currAddr |= inByte; break;
    default:
      // Note: this is probably not modelling write behaviour properly.
      logText("HAT ID writing byte 0x%0x to addr 0x%0x\n", inByte, currAddr);
      mem[currAddr & (kMemSize - 1u)] = inByte;
      // Auto-increment address; addressing wraps at the end of the page.
      currAddr++;
      if (!(currAddr & kPageSize)) currAddr -= kPageSize;
      break;
  }
  byteCount++;
  // Byte accepted, send ACK.
  return true;
}

// Read a byte of data from the RPi Sense HAT ID EEPROM.
bool i2c_hat_id::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  outByte = mem[currAddr & (kMemSize - 1u)];
  oobOut = 0u;
  logText("HAT ID reading byte 0x%0x from addr 0x%0x\n", outByte, currAddr);
  // Auto-increment address.
  currAddr++;
  // Byte available, transmit to controller.
  return true;
}
