// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * This DPI model provides a simple implementation of a SPI-connected microSD card.
 * It provides enough functionality for the existing tests to run in simulation
 * but little more than that.
 *
 * Single Block Read/Write commands are supported and the normal initialisation
 * sequence.
 *
 * The code expects a file of the name specified to the constructor, e.g. 'sd.img',
 * and will operate on that as if it were an SD card, performing block transfers
 * to/from that file.
 */

#include <cstring>

#include "spi_microsd.hh"

// SD command codes.
enum {
  CMD_GO_IDLE_STATE        = 0,
  CMD_SEND_OP_COND         = 1,
  CMD_SEND_IF_COND         = 8,
  CMD_SEND_CSD             = 9,
  CMD_SEND_CID             = 10,
  CMD_STOP_TRANSMISSION    = 12,
  CMD_SET_BLOCKLEN         = 16,
  CMD_READ_SINGLE_BLOCK    = 17,
  CMD_READ_MULTIPLE_BLOCK  = 18,
  CMD_WRITE_SINGLE_BLOCK   = 24,
  CMD_WRITE_MULTIPLE_BLOCK = 25,
  SD_SEND_OP_COND          = 41,
  CMD_APP_CMD              = 55,
  CMD_READ_OCR             = 58,
  CMD_CRC_ON_OFF           = 59,
};

// The CI system presently does not have the means to obtain/create a suitable
// SD card image ('sd.img') for use in simulation so set this to 'false' to indicate
// the presence of a card and leave the test code to spot from the CID/CSD that it
// is running in simulation.
static const bool kMustHaveSD = false;

void spi_microsd::reset() {
  spidpi::reset();
  cmdBytes = 0u;
  responding = false;
  reading = false;
  writing = false;

  // Initialise the CID
  memset(cid, 0, sizeof(cid));
  // Provide some suggestion of valid details if we have an 'sd.img'; the CI system
  // checks the first 3 bytes in simulation to ascertain whether it should attempt
  // block transfers and filing system testing.
  if  (sd) {
    cid[0] = 0x70;
    cid[1] = 'l';
    cid[2] = 'R';
  }
  cid[15] = update_crc7(0, cid, 15);
  // Initialise the CSD
  memset(csd, 0, sizeof(csd));
  csd[15] = update_crc7(0, csd, 15);
}

void spi_microsd::writeByte(uint8_t inByte, uint32_t oobIn) {
  logText("microSD %0x\n", inByte);

  if (writing) {
    switch (writeState) {
      case WriteState_StartToken:
        if (inByte == 0xfeu) {
          writeState = WriteState_DataBytes;
          // CRC16 is seeded with all zero bits.
          writeCrc16 = 0u;
        }
        break;
      case WriteState_DataBytes:
        if (writeBytes < sizeof(writeBuf)) {
          writeBuf[writeBytes] = inByte;
        }
        writeCrc16 = update_crc16(writeCrc16, &inByte, 1u);
        if (++writeBytes >= kBlockLen) {
          writeState = WriteState_CRC0;
        }
        break;
      case WriteState_CRC0:
        if (inByte != (uint8_t)(writeCrc16 >> 8)) {
          logText("Mismatched CRC16 on write data");
        }
        writeState = WriteState_CRC1;
        break;
      case WriteState_CRC1:
        if (inByte != (uint8_t)writeCrc16) {
          logText("Mismatched CRC16 on write data");
        }
        writeState = WriteState_StartToken;
        // The data_response is 0xe5 indicating that the write data has been accepted,
        // and then '0' bytes whilst busy until Idle again.
        sendResponse(8u, ((uint64_t)0x01 << 56) | 0xe5);
        writing = false;
        // At this point we write out the data block.
        if (sd && kBlockLen != fwrite(writeBuf, 1, kBlockLen, sd)) {
          logText("Failed writing to SD card");
        }
        break;
      default:
        break;
    }
  } else {
    switch (cmdBytes) {
      case 0u:
        if ((inByte & 0xc0) == 0x40) {
          cmd.cmdCode = inByte & 0x3f;
          cmdBytes = 1u;
        }
        break;
      case 5u:
        cmd.crc = inByte;
        startCommand();
        cmdBytes = 0u;
        break;
      default:
        // Collect the next byte of the address.
        cmd.address = (cmd.address << 8) | inByte;
        cmdBytes++;
        break;
    }
  }
}

bool spi_microsd::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  if (responding) {
    if (rspBytes < rspLen) {
      outByte = rsp[rspBytes++];
      // OOB data carries SD card detect - absent (1) or present (0).
      oobOut = kMustHaveSD && !sd;
      return true;
    }
    if (reading) {
      readState = ReadState_DataBytes;
      // CRC16 is seeded with all zero bits.
      readCrc16 = 0u;
    }
    responding = false;
  }
  if (reading) {
    // Checksum?
    switch (readState) {
      case ReadState_DataBytes: {
        switch (cmd.cmdCode) {
          case CMD_SEND_CID:
            outByte = cid[readBytes];
            readCrc16 = update_crc16(readCrc16, &outByte, 1u);
            if (++readBytes >= sizeof(cid)) {
              readState = ReadState_CRC0;
            }
            break;
          case CMD_SEND_CSD:
            outByte = csd[readBytes];
            readCrc16 = update_crc16(readCrc16, &outByte, 1u);
            if (++readBytes >= sizeof(csd)) {
              readState = ReadState_CRC0;
            }
            break;
          case CMD_READ_MULTIPLE_BLOCK:
          case CMD_READ_SINGLE_BLOCK: {
            // Return the next byte from the memory.
            int ch = 0xffu;
            if (sd) {
              ch = fgetc(sd);
              if (ch == EOF) {
                logText("Failed reading from SD card");
              }
            }
            outByte = ch;
            readCrc16 = update_crc16(readCrc16, &outByte, 1u);
            if (!(++readBytes & (kBlockLen - 1u))) {
              readState = ReadState_CRC0;
            }
          }
          break;
          default:
            break;
        }
      }
      break;
      case ReadState_CRC0:
        outByte = (uint8_t)(readCrc16 >> 8);
        readState = ReadState_CRC1;
        break;
      case ReadState_CRC1:
        readBytes = 0u;
        outByte = (uint8_t)readCrc16;
        readState = ReadState_DataBytes;
        break;
      default:
        outByte = 0xffu;
        reading = false;
        break;
    }

    // OOB data carries SD card detect - absent (1) or present (0).
    oobOut = kMustHaveSD && !sd;
    logText("Read byte 0x%0x\n", outByte);
    return true;
  } else {
    return false;
  }
}

// Transition on one or more CS lines.
void spi_microsd::csChanged(bool csAsserted, uint32_t oobIn) {
  cmdBytes = 0u;
  responding = false;
  reading = false;
}

// TODO: prevent command processing trying to read from non-extant SD card;
// reject out-of-order command codes etc.
void spi_microsd::startCommand() {
  logText("Starting command %02x,%08x,%02x", cmd.cmdCode, cmd.address, cmd.crc);
  switch (cmd.cmdCode) {
    case CMD_GO_IDLE_STATE:
      sendResponse(1u, 0x01);
      break;
    case CMD_SEND_IF_COND:
      sendResponse(5u, 0);
      break;

    // Card Identification Data and Card Specific Data may be handled similarly.
    case CMD_SEND_CID:
    case CMD_SEND_CSD:
      sendResponse(2u, 0xfe00);
      reading = true;
      readState = ReadState_CmdStatus;
      readBytes = 0u;
      break;

    case CMD_STOP_TRANSMISSION:
      break;
    case CMD_SET_BLOCKLEN:
      sendResponse(1u, 0);
      break;
    case CMD_READ_MULTIPLE_BLOCK:
    case CMD_READ_SINGLE_BLOCK:
      // TODO: introduce error code responses.
      if (sd) {
        logText("Reading from block %08x", cmd.address);
        fseek(sd, cmd.address << kLog2BlockLen, SEEK_SET);
      }
      sendResponse(2u, 0xfe00);
      reading = true;
      readState = ReadState_CmdStatus;
      readBytes = 0u;
      break;
    case CMD_WRITE_MULTIPLE_BLOCK:
    case CMD_WRITE_SINGLE_BLOCK:
      // TODO: introduce error code responses.
      if (sd) {
        logText("Writing to block %08x", cmd.address);
        fseek(sd, cmd.address << kLog2BlockLen, SEEK_SET);
      }
      sendResponse(1u, 0xfe);
      writing = true;
      writeState = WriteState_StartToken;
      writeBytes = 0u;
      break;
    case SD_SEND_OP_COND:
      sendResponse(1, 0x00);
      break;
    case CMD_APP_CMD:
      sendResponse(1, 0x00);
      break;
    case CMD_READ_OCR:
      sendResponse(5u, 0);
      break;    

    default:
      // Treat anything else as an illegal command.
      sendResponse(1u, 0x05);
      break;
  }
}

void spi_microsd::sendResponse(unsigned len, uint64_t d) {
  responding = true;
  rspBytes = 0u;
  rspLen = len;
  rsp[0] = (uint8_t)d;
  rsp[1] = (uint8_t)(d >> 8);
  rsp[2] = (uint8_t)(d >> 16);
  rsp[3] = (uint8_t)(d >> 24);
  rsp[4] = (uint8_t)(d >> 32);
  rsp[5] = (uint8_t)(d >> 40);
  rsp[6] = (uint8_t)(d >> 48);
  rsp[7] = (uint8_t)(d >> 56);
}

// Update the running CRC7 value for a series of bytes (command/response);
// used to generate the CRC for a command or check that of a response if CRC_ON mode is used.
//
// Note: the CRC7 is held in the MSBs of the byte, which should be set to 0 for the first
// invocation, and then the trailing LSB is always 1.
uint8_t spi_microsd::update_crc7(uint8_t crc, const uint8_t *data, size_t len) {
  while (len-- > 0u) {
    uint8_t d = *data++;
    for (unsigned b = 0u; b < 8u; b++) {
      crc = (crc << 1) ^ (((crc ^ d) & 0x80u) ? 0x12u : 0u);
      d <<= 1;
    }
  }
  // 7 MSBs contain the CRC residual and the trailing bit is 1.
  return (crc | 1u);
}

// Calculate the CRC16 value for a series of bytes (data blocks);
// used for generation or checking, if CRC_ON mode is used.
//
// Note: the CRC shall be zero for the first invocation.
uint16_t spi_microsd::update_crc16(uint16_t crc, const uint8_t *data, size_t len) {
  while (len-- > 0u) {
    uint16_t d = (uint16_t)*data++ << 8;
    for (unsigned b = 0u; b < 8u; b++) {
      crc = (crc << 1) ^ (((crc ^ d) & 0x8000u) ? 0x1021u : 0u);
      d <<= 1;
    }
  }
  return crc;
}
