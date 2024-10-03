/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi.h"

#include <stdint.h>

#include "dev_access.h"
#include "sonata_system.h"

// App flash SPI is connected to CS line 0.
const uint8_t SpiCsLine = 0;

uint8_t CmdReadJEDECId         = 0x9f;
uint8_t CmdWriteEnable         = 0x06;
uint8_t CmdSectorErase         = 0x20;
uint8_t CmdReadStatusRegister1 = 0x05;
uint8_t CmdPageProgram         = 0x02;
uint8_t CmdReadData            = 0x03;

void flash_erase_sector(uint32_t sector_idx) {
  uint8_t erase_cmd[4] = {CmdSectorErase, (sector_idx >> 16) & 0xff, (sector_idx >> 8) & 0xff, sector_idx & 0xff};

  spi_t spi;
  spi_init(&spi, FLASH_SPI, 0 /* speed, currently unused */);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, &CmdWriteEnable, 1);
  spi_set_cs(&spi, SpiCsLine, 1);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, erase_cmd, 4);
  spi_set_cs(&spi, SpiCsLine, 1);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, &CmdReadStatusRegister1, 1);

  uint8_t status;
  do {
    spi_rx(&spi, &status, 1);
  } while ((status & 0x1) == 1);

  spi_set_cs(&spi, SpiCsLine, 1);
}

void flash_write_page(uint32_t page_idx, uint8_t* data) {
  uint8_t write_cmd[4] = {CmdPageProgram, (page_idx >> 16) & 0xff, (page_idx >> 8) & 0xff, page_idx & 0xff};

  spi_t spi;
  spi_init(&spi, FLASH_SPI, 0 /* speed, currently unused */);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, &CmdWriteEnable, 1);
  spi_set_cs(&spi, SpiCsLine, 1);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, write_cmd, 4);
  spi_tx(&spi, data, 256);
  spi_set_cs(&spi, SpiCsLine, 1);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, &CmdReadStatusRegister1, 1);

  uint8_t status;
  do {
    spi_rx(&spi, &status, 1);
  } while ((status & 0x1) == 1);

  spi_set_cs(&spi, SpiCsLine, 1);
}

void flash_read(uint32_t address, uint8_t* data_out, uint32_t len) {
  uint8_t read_cmd[4] = {CmdReadData, (address >> 16) & 0xff, (address >> 8) & 0xff, address & 0xff};

  spi_t spi;
  spi_init(&spi, FLASH_SPI, 0 /* speed, currently unused */);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, read_cmd, 4);
  spi_rx(&spi, data_out, len);
  spi_set_cs(&spi, SpiCsLine, 1);
}

void spi_test() {
  uint8_t jedec_data[3];

  spi_t spi;
  spi_init(&spi, FLASH_SPI, 0 /* speed, currently unused */);

  spi_set_cs(&spi, SpiCsLine, 0);
  spi_tx(&spi, &CmdReadJEDECId, 1);
  spi_rx(&spi, jedec_data, 3);
  spi_set_cs(&spi, SpiCsLine, 1);

  putstr("Got JEDEC data ");
  puthex(jedec_data[0]);
  putchar(' ');
  puthex(jedec_data[1]);
  putchar(' ');
  puthex(jedec_data[2]);
  putstr("\r\n");
}

uint8_t write_data[256];
uint8_t read_data[256];

int main(void) {
  for (int i = 0; i < 256; ++i) {
    write_data[i] = i;
  }

  uart_init(DEFAULT_UART);
  putstr("Hello world\r\n");
  spi_test();

  flash_erase_sector(0);
  flash_write_page(0, write_data);
  flash_read(0, read_data, 256);

  putstr("Got first flash read:\r\n");
  for (int i = 0; i < 256; ++i) {
    puthex(read_data[i]);
    putstr("\r\n");
  }

  flash_read(128, read_data, 256);

  putstr("Got second flash read:\r\n");
  for (int i = 0; i < 256; ++i) {
    puthex(read_data[i]);
    putstr("\r\n");
  }

  return 0;
}
