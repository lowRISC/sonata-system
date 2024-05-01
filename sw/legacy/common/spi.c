// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "spi.h"

#include <stdint.h>

#include "dev_access.h"

void spi_init(spi_t *spi, spi_reg_t spi_reg, uint32_t speed) {
  spi->reg   = spi_reg;
  spi->speed = speed;
}

void spi_wait_idle(spi_t *spi) {
  while((DEV_READ(spi->reg + SPI_STATUS) & 0x40000) == 0);
}

void spi_tx(spi_t *spi, const uint8_t* data, uint32_t len) {
  spi_wait_idle(spi);

  // TX_ENABLE
  DEV_WRITE(spi->reg + SPI_CONTROL, 0x4);
  DEV_WRITE(spi->reg + SPI_START, len);

  uint32_t tx_avail = 0;
  for (int i = 0;i < len; ++i) {
    if (tx_avail == 0) {
      while (tx_avail < 120) {
        tx_avail = 128 - (DEV_READ(spi->reg + SPI_STATUS) & 0xff);
      }
    }

    DEV_WRITE(spi->reg + SPI_TX_FIFO, data[i]);
    tx_avail--;
  }
}

void spi_rx(spi_t *spi, uint8_t* data, uint32_t len) {
  spi_wait_idle(spi);

  // RX_ENABLE
  DEV_WRITE(spi->reg + SPI_CONTROL, 0x8);
  DEV_WRITE(spi->reg + SPI_START, len);

  for (int i = 0;i < len; ++i) {
    while (((DEV_READ(spi->reg + SPI_STATUS) >> 8) & 0xff) == 0);
    data[i] = (uint8_t)DEV_READ(spi->reg + SPI_RX_FIFO);;
  }
}
