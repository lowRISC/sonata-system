// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef SPI_H__
#define SPI_H__

#include <stdbool.h>
#include <stdint.h>

#define SPI_CFG 0xc
#define SPI_CONTROL 0x10
#define SPI_STATUS 0x14
#define SPI_START 0x18
#define SPI_RX_FIFO 0x1c
#define SPI_TX_FIFO 0x20
#define SPI_INFO 0x24
#define SPI_CS 0x28

#define SPI_FROM_BASE_ADDR(addr) ((spi_reg_t)(addr))

typedef void *spi_reg_t;
typedef struct spi {
  spi_reg_t reg;
  uint32_t speed;
} spi_t;

void spi_init(spi_t *spi, spi_reg_t reg, uint32_t speed);

void spi_wait_idle(spi_t *spi);
void spi_set_cs(spi_t *spi, uint8_t cs_line, bool cs_level);
void spi_tx(spi_t *spi, const uint8_t *data, uint32_t len);
void spi_rx(spi_t *spi, uint8_t *data, uint32_t len);

#endif  // SPI_H__
