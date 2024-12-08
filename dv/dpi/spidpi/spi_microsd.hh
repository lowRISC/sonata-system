// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <stdio.h>
#include "spidpi.hh"

// -------------------------- SPI microSD model -------------------------------
class spi_microsd : public spidpi {
public:
  spi_microsd(unsigned dataW,      // Number of data lines.
              unsigned oobInW,     // Width of Out-Of-Band input data (bits).
              unsigned oobOutW,    // Width of Out-Of-Band output data (bits).
              const char *sdFile,  // Filename of the SD card image.
              bool log = false) :  // Enable diagnostic logging?
              spidpi(dataW, oobInW, oobOutW, log) {
      assert(sdFile);
      logText("microSD model attempting to open image '%s'\n", sdFile);
      sd = fopen(sdFile, "r+b");
      // If the open fails, it is not an error at this point; it is treated as analogous
      // to there being no usable SD card present.
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

  // SD command received.
  virtual void startCommand();

  // Send response from SD card.
  virtual void sendResponse(unsigned len, uint64_t d); // Up to 7 bytes.

private:
  // SPI mode uses only 512-byte blocks, so we choose this as the block size.
  static constexpr unsigned kLog2BlockLen = 9u;
  static constexpr unsigned kBlockLen = 1u << kLog2BlockLen;

  // SD card commands are 48 bits (6 bytes) with the data being transmitted MSB first.
  struct {
    uint8_t  cmdCode;
    uint32_t address;
    uint8_t  crc;
  } cmd;

  // The number of bytes thus far received.
  uint8_t cmdBytes;

  // Responding to a command?
  bool responding;
  // sendResponse can supply 8 bytes.
  uint8_t rsp[8u];
  unsigned rspBytes;
  unsigned rspLen;

  // Reading data from the microSD card?
  bool reading;
  enum {
    ReadState_CmdStatus,
    ReadState_DataBytes,
    ReadState_CRC0,
    ReadState_CRC1
  } readState;
  // Number of bytes returned within the current data packet.
  uint32_t readBytes;
  // CRC16 for read data block.
  uint16_t readCrc16;

  // Writing data to the microSD card?
  bool writing;
  enum {
    WriteState_StartToken,
    WriteState_DataBytes,
    WriteState_CRC0,
    WriteState_CRC1
  } writeState;
  // Number of bytes written within the current data packet.
  uint32_t writeBytes;
  // Collected write data for validation before anything is written.
  uint8_t writeBuf[kBlockLen];
  // CRC16 for write data block; updated as the bytes are received.
  uint16_t writeCrc16;

  // Card Identification Data, including trailing CRC7 byte.
  uint8_t cid[16];
  // Card Specific Data, including trailing CRC7 byte.
  uint8_t csd[16];

  // Storage medium (raw microSD card image; format and contents unspecified).
  // If the file handle is NULL this means that there is no microSD card present.
  FILE *sd;

  // Utility functions for performing a running update of CRC values.
  static uint8_t  update_crc7(uint8_t crc, const uint8_t *data, size_t len);
  static uint16_t update_crc16(uint16_t crc, const uint8_t *data, size_t len);
};
