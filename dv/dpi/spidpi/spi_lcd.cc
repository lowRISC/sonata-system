// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spi_lcd.hh"

// --------
// TODO: Simply copied from lcd_std7735_cmds.h for now
// --------
typedef enum {
  ST7735_NOP     = 0x00,
  ST7735_SWRESET = 0x01,
  ST7735_RDDID   = 0x04,
  ST7735_RDDST   = 0x09,
  ST7735_SLPIN   = 0x10,
  ST7735_SLPOUT  = 0x11,
  ST7735_PTLON   = 0x12,
  ST7735_NORON   = 0x13,
  ST7735_INVOFF  = 0x20,
  ST7735_INVON   = 0x21,
  ST7735_DISPOFF = 0x28,
  ST7735_DISPON  = 0x29,
  ST7735_CASET   = 0x2A,
  ST7735_RASET   = 0x2B,
  ST7735_RAMWR   = 0x2C,
  ST7735_RAMRD   = 0x2E,
  ST7735_PTLAR   = 0x30,
  ST7735_COLMOD  = 0x3A,
  ST7735_MADCTL  = 0x36,
  ST7735_FRMCTR1 = 0xB1,
  ST7735_FRMCTR2 = 0xB2,
  ST7735_FRMCTR3 = 0xB3,
  ST7735_INVCTR  = 0xB4,
  ST7735_DISSET5 = 0xB6,
  ST7735_PWCTR1  = 0xC0,
  ST7735_PWCTR2  = 0xC1,
  ST7735_PWCTR3  = 0xC2,
  ST7735_PWCTR4  = 0xC3,
  ST7735_PWCTR5  = 0xC4,
  ST7735_VMCTR1  = 0xC5,
  ST7735_RDID1   = 0xDA,
  ST7735_RDID2   = 0xDB,
  ST7735_RDID3   = 0xDC,
  ST7735_RDID4   = 0xDD,
  ST7735_PWCTR6  = 0xFC,
  ST7735_GMCTRP1 = 0xE0,
  ST7735_GMCTRN1 = 0xE1,
} ST7735_Cmd;

typedef enum {
  ST77_MADCTL_MX  = 0x01 << 7,  // Column Address Order
  ST77_MADCTL_MV  = 0x01 << 6,  // Row/Column Exchange
  ST77_MADCTL_MY  = 0x01 << 5,  // Row Address Order
  ST77_MADCTL_ML  = 0x01 << 4,
  ST77_MADCTL_RGB = 0x01 << 3,
  ST77_MADCTL_MH  = 0x01 << 2
} ST77_MADCTL_Bits;

// Color definitions
typedef enum {
  ST7735_ColorBlack   = 0x0000,
  ST7735_ColorBlue    = 0x001F,
  ST7735_ColorRed     = 0xF800,
  ST7735_ColorGreen   = 0xE007,
  ST7735_ColorCyan    = 0x07FF,
  ST7735_ColorMagenta = 0xF81F,
  ST7735_ColorYellow  = 0xFFE0,
  ST7735_ColorWhite   = 0xFFFF,
} ST7735_Color;
// --------

void spi_lcd::reset() {
  spidpi::reset();

  cmdLen = 0u;
  xEnd = xStart = 0u;
  yEnd = yStart = 0u;
  colAdr = 0u;
  rowAdr = 0u;
  pixWriting = false;
  memset(frameBuf, 0u, sizeof(frameBuf));

  imgFilename = "lcd.ppm";
  imgLatestTime = time(nullptr);

  outputImage();
}

// Output the LCD image to a PPM file using the supplied filename.
void spi_lcd::outputImage() {
  FILE *out = fopen(imgFilename.c_str(), "wb");
  if (out) {
    fprintf(out, "P6 %u %u %u\n", kWidth, kHeight, 0x3fu);
    unsigned y = 0u;
    while (y < kHeight) {
      const pixel_t *row = frameBuf[y];
      unsigned x = 0;
      while (x < kWidth) {
        // Extract components from 5:6:5 format.
        unsigned r =  row[x] & 0x1fu;
        unsigned g = (row[x] >> 5) & 0x3fu;
        unsigned b =  row[x] >> 11;
        // We must use 6:6:6 output format.
        b = (b << 1) | (b >> 4);
        r = (r << 1) | (r >> 4);
        fputc(r, out); fputc(g, out); fputc(b, out);
        x++;
      }
      y++;
    }
    fclose(out);

    // Remember the time of the most recent successful image output; updating the file too
    // frequently is just wasteful, and it may also cause problems for any simple image viewer
    // that is showing the file contents.
    imgLatestTime = time(nullptr);
  }
}

void spi_lcd::writeByte(uint8_t inByte, uint32_t oobIn) {
  bool cmdComplete = true;  // Assume all unrecognised commands are a single byte long.
  bool emitImage = false;

  // Collect command bytes until we have the full command.
  cmd[cmdLen++] = inByte;
  if (pixWriting) {
    // We support only the 5:6:5 format presently, so two bytes per pixel.
    cmdComplete = (cmdLen >= 2);
    if (cmdComplete) {
      // Ensure that we do not write out-of-bounds.
      if (colAdr < kWidth && rowAdr < kHeight) {
        uint16_t pixCol = ((uint16_t)cmd[0] << 8) | cmd[1];
        frameBuf[rowAdr][colAdr] = pixCol;
      }
      // Advance to the next pixel.
      if (++colAdr > xEnd) {
        // Update the output image.
        emitImage = true;
        colAdr = xStart;
        if (++rowAdr > yEnd) {
          pixWriting = false;
        }
        logText("colAdr 0x%0x rowAdr 0x%0x\n", colAdr, rowAdr);
      }
    }
  } else {
    // If we now have a complete command then execute it.
    logText("%0x (cmdLen %0x cmd %0x)\n", inByte, cmdLen, cmd[0]);
    switch (cmd[0u]) {
      case ST7735_NOP:     cmdComplete = true; break;
      case ST7735_SWRESET: reset(); cmdComplete = true; break;
      case ST7735_SLPIN:   cmdComplete = true; break;
      case ST7735_SLPOUT:  cmdComplete = true; break;
      case ST7735_COLMOD:  cmdComplete = (cmdLen >= 2); break;
      case ST7735_FRMCTR1: cmdComplete = (cmdLen >= 4); break;
      case ST7735_FRMCTR2: cmdComplete = (cmdLen >= 4); break;
      case ST7735_FRMCTR3: cmdComplete = (cmdLen >= 7); break;
      case ST7735_MADCTL:  cmdComplete = (cmdLen >= 2); break;
      case ST7735_DISSET5: cmdComplete = (cmdLen >= 3); break;
      case ST7735_INVCTR:  cmdComplete = (cmdLen >= 2); break;
      case ST7735_PWCTR1:  cmdComplete = (cmdLen >= 3); break;
      case ST7735_PWCTR2:  cmdComplete = (cmdLen >= 2); break;
      case ST7735_PWCTR3:  cmdComplete = (cmdLen >= 3); break;
      case ST7735_PWCTR4:  cmdComplete = (cmdLen >= 3); break;
      case ST7735_PWCTR5:  cmdComplete = (cmdLen >= 3); break;
      case ST7735_PWCTR6:  cmdComplete = (cmdLen >= 3); break;
      case ST7735_VMCTR1:  cmdComplete = (cmdLen >= 3); break;

      // Display inversion.
      case ST7735_INVOFF: break;
      case ST7735_INVON:  break;

      // Gamma.
      case ST7735_GMCTRP1: cmdComplete = (cmdLen >= 17); break;
      case ST7735_GMCTRN1: cmdComplete = (cmdLen >= 17); break;

      case ST7735_CASET:
        cmdComplete = (cmdLen >= 5);
        if (cmdComplete) {
          xStart = (cmd[1] << 8) | cmd[2];
          xEnd   = (cmd[3] << 8) | cmd[4];
          logText("CASET 0x%0x,%0x\n", xStart, xEnd);
        }
        break;
      case ST7735_RASET:
        cmdComplete = (cmdLen >= 5);
        if (cmdComplete) {
          yStart = (cmd[1] << 8) | cmd[2];
          yEnd   = (cmd[3] << 8) | cmd[4];
          logText("RASET 0x%0x,%0x\n", yStart, yEnd);
        }
        break;

      case ST7735_NORON: break;

      // Display On/Off.
      case ST7735_DISPON:  break;
      case ST7735_DISPOFF: break;

      // RAM Writing.
      case ST7735_RAMWR:
        logText("RAMWR 0x%0x,%0x : 0x%0x,%0x\n", xStart, xEnd, yStart, yEnd);
        colAdr = xStart;
        rowAdr = yStart;
        pixWriting = true;
        cmdComplete = true;
        break;
      // RAM Reading.
      case ST7735_RAMRD:
        break;

      // Driver code does attempt to issue command 0x84 (source inspection),
      // because it provides another parameter to PWCTR1.
      default:
        break;
    }
  }

  // Emit an updated image?
  if (emitImage) {
    // Avoid writing twice within the same second to prevent overly-frequent updating.
    if (time(nullptr) != imgLatestTime) outputImage();
  }

  // Update command parser.
  if (cmdComplete) cmdLen = 0u;
}
