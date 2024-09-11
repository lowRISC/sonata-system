// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#include "spi_lcd.hh"
#include "spi_flash.hh"

void spidpi::reset() {
  // Write data to device (COPI).
  inByte = 0u;
  inBits = 0u;
  // Read data from device (CIPO).
  outBits = 0u;
  outByte = 0u;
  oobOut  = 0u;
}

// Sampling transition occurred on the SCK line.
// - the function is supplied with the CS, the new COPI data for a write operation as well as the
//   Out-Of-Band input signals.
void spidpi::sampleEdge(uint32_t cs, uint32_t copi, uint32_t oobIn) {
  // TODO: We support only a single data line (COPI) at present.
  // Data arrives MS bit first.
  inByte = (inByte << 1) | (copi & 1u);
  if (++inBits >= 8u) {
    writeByte(inByte, oobIn);
    inBits = 0u;
  }
}

// Launch transition has occurred on the SCK line.
// - the function is supplied with the CS, and the Out-Of-Band input signals.
// - the result sets the Out-of-Band output signals (upper bits) and the CIPO data lines (LSBs) if
//   the device is producing read data.
uint32_t spidpi::launchEdge(uint32_t cs, uint32_t oobIn) {
  if (!outBits) {
    bool bReading = readByte(oobIn, outByte, oobOut);
    outBits = bReading ? 8u : 1u;
  }

  // TODO: only one CIPO line supported presently.
  unsigned cipo = (outByte >> 7);
  outByte <<= 1;
  outBits--;
  return (oobOut << 1) | cipo;
}

// Transition on one or more CS lines.
void spidpi::csEdge(uint32_t cs, uint32_t oobIn) {
  // Inform the appropriate device(s) of the change the CS line state.
  csChanged(!(cs & 1u), oobIn);
}

// Logging utility function.
void spidpi::logText(const char *fmt, ...) {
  if (logging) {
    va_list va;
    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);
  }
}

// Interface is using vanilla C, so these functions collect the object pointer.
extern "C" {
// SPI DPI initialisation.
void *spidpi_create(const char *id,      // Bus identification.
                    unsigned ndevices,   // Number of devices on bus (=number of selects).
                    unsigned dataW,      // Number of data lines.
                    unsigned oobInW,     // Width of Out-Of-Band input data (bits).
                    unsigned oobOutW) {  // Width of Out-Of-Band output data (bits).
  spidpi *ctx = nullptr;
  // TODO: at present we attach only a single device to each SPI bus.
  assert(ndevices == 1u);
  // Attach the appropriate devices to this bus.
  if (!strcmp(id, "flash")) {
    ctx = new spi_flash(dataW, oobInW, oobOutW);
  } else if (!strcmp(id, "lcd")) {
    ctx = new spi_lcd(dataW, oobInW, oobOutW);
  } else {
    ctx = new spidpi(dataW, oobInW, oobOutW, true);
    ctx->logText("Warning: SPI bus '%s' not recognised", id);
  }
  assert(ctx);
  return (void*)ctx;
}

// SPI DPI finalisation.
void spidpi_destroy(void *ctx_v) {
  spidpi *ctx = (spidpi*)ctx_v;
  assert(ctx);
  delete ctx;
}

// Sampling transition on the SCK line.
void spidpi_sampleEdge(void *ctx_v, uint32_t cs, uint32_t copi, uint32_t oobIn) {
  spidpi *ctx = (spidpi*)ctx_v;
  assert(ctx);
  ctx->sampleEdge(cs, copi, oobIn);
}

// Launch transition on the SCK line.
uint32_t spidpi_launchEdge(void *ctx_v, uint32_t cs, uint32_t oobIn) {
  spidpi *ctx = (spidpi*)ctx_v;
  assert(ctx);
  return ctx->launchEdge(cs, oobIn);
}

// Note: This interface is presently inadequate for modelling some devices because they will need
// to know when the CS signal becomes deasserted, and there will not necessarily be a clock
// assertion whilst CS is deasserted.
void spidpi_csEdge(void *ctx_v, uint32_t cs, uint32_t oobIn) {
  spidpi *ctx = (spidpi*)ctx_v;
  assert(ctx);
  return ctx->csEdge(cs, oobIn);
}
};
