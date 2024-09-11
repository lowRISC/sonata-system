// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DV_DPI_SPIDPI_H_
#define __DV_DPI_SPIDPI_H_
#include <stdint.h>

// SPI DPI model - this model is supplied with all of the signals and information required to
// support multiple devices on a single SPI bus, but presently the Sonata system employs only a
// single device per bus. Therefore, at present, the SPI device models derive from `spidpi`
// directly.
//
// If multiple devices are required to share a single bus then a `spi_device` base class may be
// introduced and an instance of `spidpi` will handling the mapping from `cs` line to `spi_device`
// object.
class spidpi {
public:
  spidpi(unsigned dataw,    // Number of data lines.
         unsigned oobInw,   // Width of Out-Of-Band input data (bits).
         unsigned oobOutw,  // Width of Out-Of-Band output data (bits).
         bool log = false) {
    logging = log;
    reset();
  }
  virtual ~spidpi() { }

  // Sampling transition occurred on the SCK line.
  void sampleEdge(uint32_t cs, uint32_t copi, uint32_t oobIn);

  // Launch transition occurred on the SCK line.
  uint32_t launchEdge(uint32_t cs, const uint32_t oobIn);

  // Transition on one or more CS lines.
  void csEdge(uint32_t cs, uint32_t oobIn);

  // Diagnostic logging output.
  void logText(const char *fmt, ...);

protected:
  // Device reset.
  virtual void reset();

  // Write a byte to the SPI device.
  virtual void writeByte(uint8_t inByte, uint32_t oobIn) {
    // Sink consumes all write traffic.
  }

  // Read a byte of data from the SPI device.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
    // Sources no read traffic.
    outByte = 0u;
    oobOut = 0u;
    return false;
  }

  // Change in the state of the CS line; 'csAsserted' indicates the new CS line state, ie. whether
  // the device is being selected.
  virtual void csChanged(bool csAsserted, uint32_t oobIn) { }

  // Enable diagnostic logging output?
  bool logging;
private:
  // --- SPI device input ---
  uint8_t inBits;
  uint8_t inByte;

  // --- SPI device output ---
  uint8_t outBits;
  uint8_t outByte;
  // Most recent Out-Of-Band output data.
  uint32_t oobOut;
};
#endif  // __DV_DPI_SPIDPI_H_
