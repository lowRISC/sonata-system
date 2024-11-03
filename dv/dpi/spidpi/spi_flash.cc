// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <string.h>

#include "spi_flash.hh"

// Flash comand codes.
enum {
  CmdEnableReset         = 0x66,
  CmdReset               = 0x99,
  CmdReadJEDECId         = 0x9f,
  CmdWriteEnable         = 0x06,
  CmdSectorErase         = 0x20,
  CmdSectorErase4        = 0x21,
  CmdReadStatusRegister1 = 0x05,
  CmdPageProgram         = 0x02,
  CmdPageProgram4        = 0x12,
  CmdReadData            = 0x03,
  CmdReadData4           = 0x13
};

void spi_flash::reset() {
  spidpi::reset();
  bProgramming = false;
  bReading = false;
  bErasing = false;
  rspLen = 0u;
  rspIdx = 0u;
}

void spi_flash::writeByte(uint8_t inByte, uint32_t oobIn) {
  logText("Flash %0x (oobIn 0x%0x)\n", inByte, oobIn);
  if (bProgramming) {
    if (memOffset < kFlashBytes) mem[memOffset] = inByte;
    memOffset++;
  } else {
    bool cmdComplete = true;  // Assume unknown commands are just a single byte.

    cmd[cmdLen++] = inByte;
    switch (cmd[0u]) {
      case CmdEnableReset: cmdComplete = true; break;
      case CmdReset: reset(); cmdComplete = true; break;
      case CmdReadJEDECId:
        rsp[0] = static_cast<uint8_t>(jedec_id >> 16);
        rsp[1] = static_cast<uint8_t>(jedec_id >> 8);
        rsp[2] = static_cast<uint8_t>(jedec_id);
        rspLen = 3u;
        rspIdx = 0u;
        cmdComplete = true;
        break;
      case CmdWriteEnable: break;
      // TODO: The Sector Erase command should not be initiated until CS is deasserted following
      // the final byte of the command sequence; if further data is received without CS deassertion
      // then the command is not actioned.
      // Whilst erasing the device shall indicate BUSY for the appropriate interval.
      case CmdSectorErase:
        cmdComplete = (cmdLen >= 4);
        if (cmdComplete) {
          memOffset = getAddress24();
          bErasing = true;
        }
        break;
      case CmdSectorErase4:
        cmdComplete = (cmdLen >= 5);
        if (cmdComplete) {
          memOffset = getAddress32();
          bErasing = true;
        }
        break;
      case CmdReadStatusRegister1: break;
      case CmdPageProgram:
        cmdComplete = (cmdLen >= 4);
        if (cmdComplete) {
          memOffset = getAddress24();
          bProgramming = true;
        }
        break;
      case CmdPageProgram4:
        cmdComplete = (cmdLen >= 5);
        if (cmdComplete) {
          memOffset = getAddress32();
          bProgramming = true;
        }
        break;
      case CmdReadData:
        cmdComplete = (cmdLen >= 4);
        if (cmdComplete) {
          memOffset = getAddress24();
          // Reading continues until the CS line is deasserted.
          bReading = true;
        }
        break;
      case CmdReadData4:
        cmdComplete = (cmdLen >= 5);
        if (cmdComplete) {
          memOffset = getAddress32();
          // Reading continues until the CS line is deasserted.
          bReading = true;
        }
        break;
      default: cmdComplete = true;
    }
    if (cmdComplete) cmdLen = 0u;
  }
}

void spi_flash::eraseSector() {
  uint32_t addr = memOffset & ~(kPageSize - 1u);
  if (addr <= kFlashBytes - kPageSize) {
    memset((void *)&mem[addr], 0xffu, kPageSize);
  }
}

bool spi_flash::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  oobOut = 0u;  // Out-Of-Band data not required.
  if (bReading) {
    // Return the next byte from the flash memory.
    outByte = (memOffset < kFlashBytes) ? mem[memOffset] : 0xffu;
    memOffset++;
    logText("Read byte 0x%0x\n", outByte);
    return true;
  } else if (rspIdx < rspLen) {
    // Return the next byte of the response.
    outByte = rsp[rspIdx++];
    logText("Flash Read 0x%0x\n", outByte);
    return true;
  }
  return false;
}

// Transition on one or more CS lines.
void spi_flash::csChanged(bool csAsserted, uint32_t oobIn) {
  if (!csAsserted) {
    // This is the point at which sector erasing actually commences.
    if (bErasing) {
      eraseSector();
      bErasing = false;
    }
    // The flash needs to cancel programming at this point, otherwise further commands will be
    // interpreted as programming data.
    bProgramming = false;
    // Similarly, an ongoing Read Data operation shall be cancelled.
    bReading = false;
  }
}
