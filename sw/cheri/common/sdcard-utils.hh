/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <platform-spi.hh>
#include <platform-gpio.hh>

#include "console.hh"

/**
 * SPI mode SD card driver that provides basic detection of microSD card presence, block-level read access
 * to the card, and reading of the Card Identification Data (CID) and Card Specific Data (CSD).
 *
 * 'Part 1 Simplified' Physical Layer Simplified Specification: https://www.sdcard.org/downloads/pls/
 */

class SdCard {
 private:
  // Access to SPI controller.
  volatile SonataSpi *spi;
  // Access to GPIO block (required for SD card detection).
  volatile SonataGpioBase *gpio;
  // Chip select (single bit set).
  uint32_t cs;
  // SD card `detect` pin (single bit set).
  uint32_t det;
  // CRC_ON mode used?
  // - this is off by default for SPI mode, but may be enabled for error detection in both the
  //   host and the card.
  bool crcOn;
  // Access to diagnostic logging.
  Log *log;

  // We need to clock the device repeatedly at startup; this test pattern is used to ensure that
  // we keep the COPI line high and it cannot be misinterpreted as a command.
  static constexpr uint8_t ones[] = {0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu};

 public:
  // Transfers in SPI mode are always in terms of 512-byte blocks.
  static constexpr unsigned kBlockLen = 512u;

  // Supplied with SPI controller and its appropriate CS line, and a GPIO block with its input pin
  // number. The SPI controller performs the communication with the SD card and the GPIO simply
  // provides the card detection indicator (low = present).
  //
  // CRC calculation is turned on by default, although this requires the CPU to perform a little
  // bit of additional work. It may be disabled if desired. SPI mode communication may be performed
  // with or without CRC checking.
  //
  // Logging may optionally be requested.
  SdCard(volatile SonataSpi *spi_, volatile SonataGpioBase *gpio_, unsigned cs_ = 1u, unsigned det_ = 16u,
         bool crc_ = true, Log *log_ = nullptr)
      : spi(spi_), gpio(gpio_), cs(1u << cs_), det(1u << det_), crcOn(crc_), log(log_) {}

  // SD command codes. (Section 7.3.1)
  enum CmdCode {
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

  // SD Control Tokens. (Section 7.3.3)
  enum CtrlToken {
    // Start Block Token precedes data block, for all but Multiple Block Write.
    StartBlockToken = 0xfeu,
    // Start Block Token used for Multiple Block Write operations.
    StartBlockTokenMW = 0xfcu,
    // Stop Transaction Token, for Multiple Block Writes.
    StopTranToken = 0xfdu,
  };

  // Indicates whether there is an SD card present in the slot.
  bool present() const { return !(gpio->debouncedInput & det); }

  void select_card(bool enable) { spi->cs = enable ? (spi->cs & ~cs) : (spi->cs | cs); }

  // Initialise the SD card ready for use.
  bool init() {
    // Every card tried seems to be more than capable of keeping up with 20Mbps.
    constexpr unsigned kSpiSpeed = 0u;
    spi->init(false, false, true, kSpiSpeed);

    // Apparently we're required to send at least 74 SD CLK cycles with
    // the device _not_ selected before talking to it.
    spi->blocking_write(ones, 10);
    spi->wait_idle();

    select_card(true);

    // Note that this is a very stripped-down card initialisation sequence
    // that assumes SDHC version 2, so use a more recent microSD card.
    do {
      send_command(CMD_GO_IDLE_STATE, 0u);
    } while (0x01 != get_response_R1());

    send_command(CMD_SEND_IF_COND, 0x1aau);
    get_response_R3();

    // Instruct the SD card whether to check CRC values on commands.
    send_command(CMD_CRC_ON_OFF, (uint32_t)crcOn);
    get_response_R1();

    // Read supported voltage range of the card.
    send_command(CMD_READ_OCR, 0);
    get_response_R3();

    do {
      send_command(CMD_APP_CMD, 0);
      (void)get_response_R1();
      // Specify Host Capacity Support as 1.
      send_command(SD_SEND_OP_COND, 1u << 30);
    } while (0x01 & get_response_R1());

    if (log) {
      log->println("Setting block length to {}", kBlockLen);
    }

    // Read card capacity information.
    send_command(CMD_READ_OCR, 0);
    get_response_R3();

    send_command(CMD_SET_BLOCKLEN, kBlockLen);
    uint8_t rd = get_response_R1();
    if (log) {
      log->println("Response: {:#04x}", rd);
    }
    select_card(false);

    return true;
  }

  // Read Card Identification Data (CID).
  bool read_cid(uint8_t *buf, size_t len) { return read_cid_csd(CMD_SEND_CID, buf, len); }

  // Read Card Specific Data (CSD).
  bool read_csd(uint8_t *buf, size_t len) { return read_cid_csd(CMD_SEND_CSD, buf, len); }

  // Read a number of contiguous blocks from the SD card.
  // TODO: Support non-blocking operation.
  bool read_blocks(uint32_t block, uint8_t *buf, size_t num_blocks = 1u, bool blocking = true) {
    const bool multi = num_blocks > 1u;

    select_card(true);

    bool ok(true);
    for (size_t blk = 0u; blk < num_blocks; blk++) {
      if (log) {
        log->println("Reading block {}", block + blk);
      }

      if (multi) {
        // Is this the first block of the read request?
        if (!blk) {
          send_command(CMD_READ_MULTIPLE_BLOCK, block);
          (void)get_response_R1();
        }
      } else {
        send_command(CMD_READ_SINGLE_BLOCK, block + blk);
        (void)get_response_R1();
      }

      if (!collected_data(&buf[blk * kBlockLen], kBlockLen)) {
        ok = false;
        break;
      }
    }

    if (multi) {
      send_command(CMD_STOP_TRANSMISSION, 0u);
      (void)get_response_R1b();
    }

    select_card(false);
    return ok;
  }

  // Write a number of contiguous blocks from the SD card.
  // TODO: Support non-blocking operation.
  bool write_blocks(uint32_t block, uint8_t *buf, size_t num_blocks = 1u, bool blocking = true) {
    const bool multi = num_blocks > 1u;
    uint8_t crc16[2];
    crc16[1] = crc16[0] = 0xffu;  // CRC16 not required by default for SPI mode.

    select_card(true);

    bool ok(true);
    for (size_t blk = 0u; blk < num_blocks; blk++) {
      if (crcOn) {
        // CRC16 bytes follow the data block.
        uint16_t crc = calc_crc16(&buf[blk * kBlockLen], kBlockLen);
        crc16[0]     = (uint8_t)(crc >> 8);
        crc16[1]     = (uint8_t)crc;
      }
      if (log) {
        log->println("Writing block {}", block + blk);
      }

      // Note: the Start Block Token differs between Multiple Block Write commands and
      // the other data transfer commands, including the Single Block Write command.
      uint8_t start_token;
      if (multi) {
        // Is this the first block of the read request?
        if (!blk) {
          send_command(SdCard::CMD_WRITE_MULTIPLE_BLOCK, block);
          (void)get_response_R1();
        }
        start_token = StartBlockTokenMW;
      } else {
        send_command(SdCard::CMD_WRITE_SINGLE_BLOCK, block + blk);
        (void)get_response_R1();
        start_token = StartBlockToken;
      }

      spi->blocking_write(&start_token, 1u);
      spi->blocking_write(&buf[blk * kBlockLen], kBlockLen);
      spi->blocking_write(crc16, sizeof(crc16));
      // Collect data_response and wait until the card is no longer busy.
      if (5 != (0x1f & get_data_response_busy())) {
        // Data not accepted because of an error.
        ok = false;
        break;
      }
    }

    if (multi) {
      const uint8_t stop_tran_token = (uint8_t)StopTranToken;
      spi->blocking_write(&stop_tran_token, 1u);
      // The card will hold the CIPO line low whilst busy, yielding repeated 0x00 bytes,
      // but it seems to drop and raise the line at an arbitrary time with respect to
      // the '8-clock counting' logic.
      while (0x00 != get_response_byte());  // Detect falling edge.
      // Card will signal busy with zeros.
      wait_not_busy();
    }

    select_card(false);
    return ok;
  }

  // Send a command to the SD card with the supplied 32-bit argument.
  void send_command(uint8_t cmdCode, uint32_t arg) {
    uint8_t cmd[6];
    if (log) {
      log->println("Sending command {:#04x}", cmdCode);
    }

    // Apparently we need to clock 8 times before sending the command.
    //
    // TODO: This may well be an issue with not aligning read data on the previous command?
    // Without this the initialisation sequence gets stuck trying to specify HCS; the SD card
    // does not become ready.
    uint8_t dummy = 0xffu;
    spi->blocking_write(&dummy, 1u);

    cmd[0] = 0x40u | cmdCode;
    cmd[1] = (uint8_t)(arg >> 24);
    cmd[2] = (uint8_t)(arg >> 16);
    cmd[3] = (uint8_t)(arg >> 8);
    cmd[4] = (uint8_t)(arg >> 0);
    // The final byte includes the CRC7 which _must_ be valid for two special commands,
    // but normally in SPI mode CRC checking is OFF.
    if (crcOn || cmdCode == CMD_GO_IDLE_STATE || cmdCode == CMD_SEND_IF_COND) {
      cmd[5] = 1u | (calc_crc7(cmd, 5) << 1);
    } else {
      // No need to expend CPU times calculating the CRC7; it will be ignored.
      cmd[5] = 0xffu;
    }
    spi->blocking_write(cmd, sizeof(cmd));
  }

  // Attempt to collect a single response byte from the device; if it is not driving the
  // CIPO line we will read 0xff.
  uint8_t get_response_byte() {
    uint8_t r;
    read_card_data(&r, 1u);
    return r;
  }

  // Get response type R1 from the SD card.
  uint8_t get_response_R1() {
    spi->wait_idle();
    while (true) {
      uint8_t rd1 = get_response_byte();
      // Whilst there is no response we read 0xff; an actual R1 response commences
      // with a leading 0 bit (MSB).
      if (!(rd1 & 0x80u)) {
        if (log) {
          log->println("R1 {:#04x}", rd1);
        }
        return rd1;
      }
    }
  }

  // Wait until the SD card declares that it is no longer busy.
  inline void wait_not_busy() {
    while (0x00 == get_response_byte());  // Wait whilst device is busy.
  }

  // Get response type R1b from the SD card.
  uint8_t get_response_R1b() {
    spi->wait_idle();
    uint8_t rd1 = get_response_R1();
    // Card may signal busy with zero bytes.
    wait_not_busy();
    return rd1;
  }

  // Get data_response after sending a block of write data to the SD card.
  uint8_t get_data_response_busy() {
    uint8_t rd1;
    spi->wait_idle();
    do {
      rd1 = get_response_byte();
    } while ((rd1 & 0x11u) != 0x01u);
    wait_not_busy();
    return rd1;
  }

  // Get response type R3 (5 bytes) from the SD card.
  void get_response_R3() {
    volatile uint8_t rd2;
    (void)get_response_R1();
    for (int r = 0; r < 4; ++r) {
      spi->transmitFifo = 0xffu;
      spi->control      = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
      spi->start        = 1u;
      spi->wait_idle();
      while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {
      }
      rd2 = static_cast<uint8_t>(spi->receiveFifo);
    }
    // We need to ensure the FIFO reads occur, but we don't need the data presently.
    rd2 = rd2;
  }

  // Calculate the CRC7 value for a series of bytes (command/response);
  // used to generate the CRC for a command or check that of a response if CRC_ON mode is used.
  static uint8_t calc_crc7(const uint8_t *data, size_t len) {
    uint8_t crc = 0u;
    while (len-- > 0u) {
      uint8_t d = *data++;
      for (unsigned b = 0u; b < 8u; b++) {
        crc = (crc << 1) ^ (((crc ^ d) & 0x80u) ? 0x12u : 0u);
        d <<= 1;
      }
    }
    // 7 MSBs contain the CRC residual.
    return crc >> 1;
  }

  // Calculate the CRC16 value for a series of bytes (data blocks);
  // used for generation or checking, if CRC_ON mode is used.
  static uint16_t calc_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0u;
    while (len-- > 0u) {
      uint16_t d = (uint16_t)*data++ << 8;
      for (unsigned b = 0u; b < 8u; b++) {
        crc = (crc << 1) ^ (((crc ^ d) & 0x8000u) ? 0x1021u : 0u);
        d <<= 1;
      }
    }
    return crc;
  }

 private:
  // Collect an expect number of bytes into the given buffer, and return an indication of whether
  // they were collected without error.
  bool collected_data(uint8_t *buf, size_t len) {
    uint8_t crc16[2];

    // Collect the data, prepended with the Start Block Token and followed by the CRC16 bytes.
    while (StartBlockToken != get_response_byte());
    // For at least one test card we need to hold the COPI line high during the data read
    // otherwise the data becomes corrupted; the card appears to be starting to accept a new
    // command.
    read_card_data(buf, len);
    read_card_data(crc16, sizeof(crc16));

    // Shall we validate the CRC16 of the received data block?
    if (crcOn) {
      uint16_t exp_crc16 = calc_crc16(buf, len);
      uint16_t obs_crc16 = ((uint16_t)crc16[0] << 8) | crc16[1];
      if (log) {
        log->println("Read block CRC {:#06x}", obs_crc16);
        log->println("Calculated CRC {:#06x}", exp_crc16);
      }
      if (obs_crc16 != exp_crc16) {
        select_card(false);
        if (log) {
          log->println("CRC16 mismatch");
        }
        return false;
      }
    }
    return true;
  }

  // Shared implementation for CMD_SEND_CID and CMD_SEND_CSD.
  bool read_cid_csd(CmdCode cmd, uint8_t *buf, size_t len) {
    select_card(true);

    send_command(cmd, 0);
    (void)get_response_R1();
    bool ok = collected_data(buf, len);
    select_card(false);
    return ok;
  }

  /*
   * Receives `len` bytes and puts them in the `data` buffer,
   * where `len` is at most `0x7ff`, being careful to keep COPI high by
   * also transmitting repeated 0xff bytes.
   *
   * This method will block until the requested number of bytes has been seen.
   * There is currently no timeout.
   *
   * Note that unlike the 'blocking_read' member function of the SPI object,
   * this function intentionally keeps the COPI line high by supplying a 0xff
   * byte for each byte read. This prevents COPI line dropping and being
   * misinterpreted as the start of a command.
   */
  void read_card_data(uint8_t data[], uint16_t len) {
    assert(len <= 0x7ff);
    len &= SonataSpi::StartByteCountMask;
    spi->wait_idle();
    // Do not attempt a zero-byte transfer; not supported by the controller.
    if (len) {
      spi->control = SonataSpi::ControlReceiveEnable | SonataSpi::ControlTransmitEnable;
      spi->start   = len;

      // In addition to keeping the COPI line high to avoid inducing failures we want
      // to keep the Tx FIFO populated so that the SPI controller remains active despite
      // having double the traffic between CPU and SPI (Tx and Rx).
      //
      // This expression of the code very nearly keeps the SPI controller running at
      // SysClkFreq/2 Mbps whilst reading data, so that the data block is retrieved as
      // quickly as possible.
      //
      // Prompt the retrieval of the first byte.
      spi->transmitFifo  = 0xffu;  // Keep COPI high, staying one byte ahead of reception.
      const uint8_t *end = data + len - 1;
      while (data < end) {
        // Prompt the retrieval of the next byte.
        spi->transmitFifo = 0xffu;  // COPI high for the next byte.
        // Wait until it's available, and quickly capture it.
        while (spi->status & SonataSpi::StatusRxFifoEmpty) {
        }
        *data++ = static_cast<uint8_t>(spi->receiveFifo);
      }
      // Collect the final byte.
      while (spi->status & SonataSpi::StatusRxFifoEmpty) {
      }
      *data++ = static_cast<uint8_t>(spi->receiveFifo);
    }
  }
};
