/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>
#include <platform-spi.hh>

#include "sonata-devices.hh"
#include "timer-utils.hh"

static const uint8_t CmdEnableReset         = 0x66;
static const uint8_t CmdReset               = 0x99;
static const uint8_t CmdReadJEDECId         = 0x9f;
static const uint8_t CmdWriteEnable         = 0x06;
static const uint8_t CmdSectorErase         = 0x20;
static const uint8_t CmdSectorErase4Addr    = 0x21;
static const uint8_t CmdReadStatusRegister1 = 0x05;
static const uint8_t CmdPageProgram         = 0x02;
static const uint8_t CmdPageProgram4Addr    = 0x12;
static const uint8_t CmdReadData            = 0x03;
static const uint8_t CmdReadData4Addr       = 0x13;

class SpiFlash {
 private:
  SpiPtr spi;

  void set_cs(bool enable) { spi->chipSelects = enable ? (spi->chipSelects & ~1u) : (spi->chipSelects | 1u); }

 public:
  SpiFlash(SpiPtr spi_) : spi(spi_) {}

  void reset() {
    set_cs(true);
    spi->blocking_write(&CmdEnableReset, 1);
    spi->wait_idle();
    set_cs(false);

    set_cs(true);
    spi->blocking_write(&CmdReset, 1);
    spi->wait_idle();
    set_cs(false);

    // Need to wait at least 30us for the reset to complete.
    wait_mcycle(2000);
  }

  void read_jedec_id(uint8_t *jedec_id_out) {
    set_cs(true);
    spi->blocking_write(&CmdReadJEDECId, 1);
    spi->blocking_read(jedec_id_out, 3);
    set_cs(false);
  }

  void erase_sector(uint32_t address) {
    const uint8_t erase_cmd[5] = {CmdSectorErase4Addr, uint8_t((address >> 24) & 0xff), uint8_t((address >> 16) & 0xff),
                                  uint8_t((address >> 8) & 0xff), uint8_t(address & 0xff)};

    set_cs(true);
    spi->blocking_write(&CmdWriteEnable, 1);
    spi->wait_idle();
    set_cs(false);

    set_cs(true);
    spi->blocking_write(erase_cmd, 5);
    spi->wait_idle();
    set_cs(false);

    set_cs(true);
    spi->blocking_write(&CmdReadStatusRegister1, 1);

    uint8_t status;
    do {
      spi->blocking_read(&status, 1);
    } while ((status & 0x1) == 1);

    set_cs(false);
  }

  void write_page(uint32_t address, uint8_t *data) {
    const uint8_t write_cmd[5] = {CmdPageProgram4Addr, uint8_t((address >> 24) & 0xff), uint8_t((address >> 16) & 0xff),
                                  uint8_t((address >> 8) & 0xff), uint8_t(address & 0xff)};

    set_cs(true);
    spi->blocking_write(&CmdWriteEnable, 1);
    spi->wait_idle();
    set_cs(false);

    set_cs(true);
    spi->blocking_write(write_cmd, 5);
    spi->blocking_write(data, 256);
    spi->wait_idle();
    set_cs(false);

    set_cs(true);
    spi->blocking_write(&CmdReadStatusRegister1, 1);

    uint8_t status;
    do {
      spi->blocking_read(&status, 1);
    } while ((status & 0x1) == 1);

    set_cs(false);
  }

  void read(uint32_t address, uint8_t *data_out, uint32_t len) {
    const uint8_t read_cmd[5] = {CmdReadData4Addr, uint8_t((address >> 24) & 0xff), uint8_t((address >> 16) & 0xff),
                                 uint8_t((address >> 8) & 0xff), uint8_t(address & 0xff)};
    set_cs(true);
    spi->blocking_write(read_cmd, 5);
    spi->blocking_read(data_out, len);
    set_cs(false);
  }
};
