// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <time.h>

#include <string>

#include "spidpi.hh"

// -------------------- ST7735 LCD model --------------------
class spi_lcd : public spidpi {
public:
  spi_lcd(unsigned dataW,      // Number of data lines.
          unsigned oobInW,     // Width of Out-Of-Band input data (bits).
          unsigned oobOutW) :  // Width of Out-Of-Band output data (bits).
          spidpi(dataW, oobInW, oobOutW) {
    reset();
  }

protected:
  // Device reset.
  virtual void reset();

  // Write a byte to the SPI LCD.
  virtual void writeByte(uint8_t inByte, uint32_t oobIn);

  // Read a byte of data from the SPI LCD; reading presently not supported,
  // so leave the base class to handle CIPO lines.

  // No need to be informed of CS changes.
private:
  // TODO: we do not concern ourselves with orientation presently! tut!
  static constexpr unsigned kWidth  = 160u;
  static constexpr unsigned kHeight = 128u;
  typedef uint16_t pixel_t;

  // Write the LCD image out to the given file if enough time has elapsed since the most
  // recent update.
  void outputImage();

  // Filename of the output image file.
  std::string imgFilename;
  time_t imgLatestTime;

  bool pixWriting;  // Pixel writing mode?

  uint16_t colAdr;  // Current column.
  uint16_t rowAdr;  // Current row.

  uint16_t xStart;  // Start column.
  uint16_t xEnd;    // End column.
  uint16_t yStart;  // Start row.
  uint16_t yEnd;    // End row.
  pixel_t pixCol;   // Current pixel colour.

  // Collecting current command; none of the commands is more than 17 bytes.
  uint8_t cmdLen;
  uint8_t cmd[0x20u];

  // Pixel buffer RAM.
  pixel_t frameBuf[kHeight][kWidth];
};
