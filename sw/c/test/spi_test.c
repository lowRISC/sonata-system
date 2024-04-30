#include <stdint.h>
#include "sonata_system.h"

#define DEV_WRITE(addr, val) (*((volatile uint32_t *)(addr)) = val)
#define DEV_READ(addr) (*((volatile uint32_t *)(addr)))

#define SPI_BASE 0x83000000
#define SPI_CFG 0xc
#define SPI_CONTROL 0x10
#define SPI_STATUS 0x14
#define SPI_START 0x18
#define SPI_RX_FIFO 0x1c
#define SPI_TX_FIFO 0x20

#define GPIO_BASE 0x80000000

void spi_wait_idle() {
  while((DEV_READ(SPI_BASE + SPI_STATUS) & 0x40000) == 0);
}

void spi_tx(uint8_t* data, uint32_t len) {
  spi_wait_idle();
  // TX_ENABLE
  DEV_WRITE(SPI_BASE + SPI_CONTROL, 0x4);
  DEV_WRITE(SPI_BASE + SPI_START, len);

  uint32_t tx_avail = 0;
  for (int i = 0;i < len; ++i) {
    if (tx_avail == 0) {
      while (tx_avail < 120) {
        tx_avail = 128 - (DEV_READ(SPI_BASE + SPI_STATUS) & 0xff);
      }
    }

    DEV_WRITE(SPI_BASE + SPI_TX_FIFO, data[i]);
    tx_avail--;
  }
}

void spi_rx(uint8_t* data, uint32_t len) {
  spi_wait_idle();
  // RX_ENABLE
  DEV_WRITE(SPI_BASE + SPI_CONTROL, 0x8);
  DEV_WRITE(SPI_BASE + SPI_START, len);

  for (int i = 0;i < len; ++i) {
    while (((DEV_READ(SPI_BASE + SPI_STATUS) >> 8) & 0xff) == 0);
    data[i] = (uint8_t)DEV_READ(SPI_BASE + SPI_RX_FIFO);;
  }
}

void spi_csn(uint32_t csn) {
  DEV_WRITE(GPIO_BASE, (csn & 1) << 12);
}

uint8_t CmdReadJEDECId = 0x9f;
uint8_t CmdWriteEnable = 0x06;
uint8_t CmdSectorErase = 0x20;
uint8_t CmdReadStatusRegister1 = 0x05;
uint8_t CmdPageProgram = 0x02;
uint8_t CmdReadData = 0x03;

void flash_erase_sector(uint32_t sector_idx) {
  uint8_t erase_cmd[4] = {CmdSectorErase, (sector_idx >> 16) & 0xff,
    (sector_idx >> 8) & 0xff, sector_idx & 0xff};

  spi_csn(0);
  spi_tx(&CmdWriteEnable, 1);
  spi_csn(1);

  spi_csn(0);
  spi_tx(erase_cmd, 4);
  spi_csn(1);

  spi_csn(0);
  spi_tx(&CmdReadStatusRegister1, 1);

  uint8_t status;
  do {
    spi_rx(&status, 1);
  } while ((status & 0x1) == 1);

  spi_csn(1);
}

void flash_write_page(uint32_t page_idx, uint8_t* data) {
  uint8_t write_cmd[4] = {CmdPageProgram, (page_idx >> 16) & 0xff,
    (page_idx >> 8) & 0xff, page_idx & 0xff};

  spi_csn(0);
  spi_tx(&CmdWriteEnable, 1);
  spi_csn(1);

  spi_csn(0);
  spi_tx(write_cmd, 4);
  spi_tx(data, 256);
  spi_csn(1);

  spi_csn(0);
  spi_tx(&CmdReadStatusRegister1, 1);

  uint8_t status;
  do {
    spi_rx(&status, 1);
  } while ((status & 0x1) == 1);

  spi_csn(1);
}

void flash_read(uint32_t address, uint8_t* data_out, uint32_t len) {
  uint8_t read_cmd[4] = {CmdReadData, (address >> 16) & 0xff,
    (address >> 8) & 0xff, address & 0xff};

  spi_csn(0);
  spi_tx(read_cmd, 4);
  spi_rx(data_out, len);
  spi_csn(1);
}

void spi_test() {
  uint8_t jedec_data[3];

  spi_csn(0);
  spi_tx(&CmdReadJEDECId, 1);
  spi_rx(jedec_data, 3);
  spi_csn(1);

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

  spi_csn(1);
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

