// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "spidpi.hh"

// ----------------------- SPI Flash model ---------------------
class spi_flash : public spidpi {
public:
  spi_flash(unsigned dataW,      // Number of data lines.
            unsigned oobInW,     // Width of Out-Of-Band input data (bits).
            unsigned oobOutW) :  // Width of Out-Of-Band output data (bits).
            spidpi(dataW, oobInW, oobOutW) {
      reset();
   }

protected:
  // Device reset.
  virtual void reset();

  // Write a byte to the SPI flash.
  virtual void writeByte(uint8_t inByte, uint32_t oobIn);

  // Read a byte of data from the SPI flash.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut);

  // Change in the state of the CS line.
  virtual void csChanged(bool csAsserted, uint32_t oobIn);

private:
  // Get a 24-bit address from the command buffer at the specified offset.
  inline uint32_t getAddress24(unsigned offset = 1u) const {
    const uint8_t *addr = &cmd[offset];
    return ((uint32_t)addr[0u] << 16) | ((uint32_t)addr[1u] << 8) | addr[2u];
  }

  // Get a 32-bit address from the command buffer at the specified offset.
  inline uint32_t getAddress32(unsigned offset = 1u) const {
    const uint8_t *addr = &cmd[offset];
    return ((uint32_t)addr[0u] << 24) | ((uint32_t)addr[1u] << 16) | ((uint32_t)addr[2u] << 8) |
           addr[3u];
  }

  static constexpr unsigned kFlashBytes = 32 * 1024 * 1024;  // 256Mib.
  static constexpr unsigned kPageSize = 4 * 1024;  // 4KiB.

  void    writeFlash(uint8_t inByte, uint32_t oobIn);
  bool    readFlash(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut);
  void    eraseSector();

  bool     bProgramming;  // Programming a page?
  bool     bReading;      // Reading a sequence of bytes?
  bool     bErasing;      // Sector Erase requested?

  // Collecting current command; none of the commands is more than 5 bytes.
  uint8_t cmdLen;
  uint8_t cmd[8u];

  // Response buffer holds short responses from the flash device.
  uint8_t  rspLen;
  uint8_t  rspIdx;
  uint8_t  rsp[0x10];

  // Current offset within flash for programming/reading operation.
  uint32_t memOffset;
  uint8_t  mem[kFlashBytes];
};
