// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "ksz8851.h"

#include <lwip/etharp.h>
#include <lwip/netif.h>
#include <string.h>

#include "rv_plic.h"
#include "sonata_system.h"
#include "spi.h"
#include "timer.h"

enum {
  // IRQ
  EthIntrIrq = 47,

  // GPIO Output
  EthRstPin = 14,

  // CS line 0 on SPI controller.
  EthCsLine = 0,
};

static struct netif *eth_netif;

static void timer_delay(uint32_t ms) {
  // Configure timer to trigger every 1 ms
  timer_enable(SYSCLK_FREQ / 1000);
  uint32_t timeout = get_elapsed_time() + ms;
  while (get_elapsed_time() < timeout) {
    asm volatile("wfi");
  }
  timer_disable();
}

static uint16_t ksz8851_reg_read(spi_t *spi, uint8_t reg) {
  uint8_t be = (reg & 0x2) == 0 ? 0b0011 : 0b1100;
  uint8_t bytes[2];
  bytes[0] = (0b00 << 6) | (be << 2) | (reg >> 6);
  bytes[1] = (reg << 2) & 0b11110000;

  spi_set_cs(spi, EthCsLine, 0);
  spi_tx(spi, bytes, 2);
  uint16_t val;
  spi_rx(spi, (uint8_t *)&val, 2);
  spi_set_cs(spi, EthCsLine, 1);
  return val;
}

static void ksz8851_reg_write(spi_t *spi, uint8_t reg, uint16_t val) {
  uint8_t be = (reg & 0x2) == 0 ? 0b0011 : 0b1100;
  uint8_t bytes[2];
  bytes[0] = (0b01 << 6) | (be << 2) | (reg >> 6);
  bytes[1] = (reg << 2) & 0b11110000;

  spi_set_cs(spi, EthCsLine, 0);
  spi_tx(spi, bytes, 2);
  spi_tx(spi, (uint8_t *)&val, 2);
  spi_wait_idle(spi);
  spi_set_cs(spi, EthCsLine, 1);
}

static void ksz8851_reg_set(spi_t *spi, uint8_t reg, uint16_t mask) {
  uint16_t old = ksz8851_reg_read(spi, reg);
  ksz8851_reg_write(spi, reg, old | mask);
}

static void ksz8851_reg_clear(spi_t *spi, uint8_t reg, uint16_t mask) {
  uint16_t old = ksz8851_reg_read(spi, reg);
  ksz8851_reg_write(spi, reg, old & ~mask);
}

static void ksz8851_write_mac(spi_t *spi, struct eth_addr addr) {
  struct eth_addr eth_addr;

  ksz8851_reg_write(spi, ETH_MARH, (addr.addr[0] << 8) | addr.addr[1]);
  ksz8851_reg_write(spi, ETH_MARM, (addr.addr[2] << 8) | addr.addr[3]);
  ksz8851_reg_write(spi, ETH_MARL, (addr.addr[4] << 8) | addr.addr[5]);
}

static void ksz8851_dump(spi_t *spi) {
  uint16_t phy_status = ksz8851_reg_read(spi, ETH_P1MBSR);
  putstr("PHY status is ");
  puthexn(phy_status, 4);
  puts("");

  uint16_t port_status = ksz8851_reg_read(spi, ETH_P1SCLMD);
  putstr("Port special status is ");
  puthexn(port_status, 4);
  puts("");

  port_status = ksz8851_reg_read(spi, ETH_P1SR);
  putstr("Port status is ");
  puthexn(port_status, 4);
  puts("");

  uint16_t isr_status = ksz8851_reg_read(spi, ETH_ISR);
  putstr("ISR status is ");
  puthexn(isr_status, 4);
  puts("");

  uint16_t p1cr = ksz8851_reg_read(spi, ETH_P1CR);
  putstr("P1CR status is ");
  puthexn(p1cr, 4);
  puts("");

  uint16_t p1mbcr = ksz8851_reg_read(spi, ETH_P1MBCR);
  putstr("P1MBCR status is ");
  puthexn(p1mbcr, 4);
  puts("");
}

static err_t ksz8851_output(struct netif *netif, struct pbuf *buf) {
  spi_t *spi = netif->state;

#ifdef DEBUG
  putstr("KSZ8851: Transmitting ");
  puthexn(buf->tot_len, 4);
  puts(" bytes");
#endif

  // Wait until transmit buffer is available.
  while (1) {
    uint16_t txmir = ksz8851_reg_read(spi, ETH_TXMIR) & 0x0FFF;
    if (txmir < buf->tot_len + 4) {
#ifdef DEBUG
      puts("KSZ8851: Transmit buffer full");
#endif
      continue;
    }
    break;
  }

  // Disable IRQ to avoid interrupting DMA transfer.
  uint32_t flags = arch_local_irq_save();

  // Start QMU DMA transfer operation
  ksz8851_reg_set(spi, ETH_RXQCR, StartDmaAccess);

  // Start transmission.
  uint8_t cmd = 0b11 << 6;
  spi_set_cs(spi, EthCsLine, 0);
  spi_tx(spi, &cmd, 1);

  uint32_t header = 0x8000 | (buf->tot_len << 16);
  spi_tx(spi, (uint8_t *)&header, 4);

  uint32_t len = buf->tot_len;
  for (struct pbuf *p = buf; len != 0 && p != NULL; p = p->next) {
    uint32_t frag_len = p->len > len ? len : p->len;
    spi_tx(spi, p->payload, frag_len);
    len -= frag_len;
  }

  static const uint8_t padding[3] = {0, 0, 0};
  // The transmission needs to be dword-aligned, so we pad the packet to 4 bytes.
  uint32_t pad = (-buf->tot_len) & 0x3;
  if (pad != 0) {
    spi_tx(spi, padding, pad);
  }

  spi_wait_idle(spi);
  spi_set_cs(spi, EthCsLine, 1);

  // Stop QMU DMA transfer operation
  ksz8851_reg_clear(spi, ETH_RXQCR, StartDmaAccess);

  // TxQ Manual-Enqueue
  ksz8851_reg_set(spi, ETH_TXQCR, ManualEnqueueTxQFrameEnable);

  arch_local_irq_restore(flags);

  return 0;
}

static void ksz8851_drop_error_frame(spi_t *spi) {
  ksz8851_reg_set(spi, ETH_RXQCR, ReleaseRxErrorFrame);

  // Wait until the frame is dropped.
  while (ksz8851_reg_read(spi, ETH_RXQCR) & ReleaseRxErrorFrame);
}

static void ksz8851_recv(struct netif *netif) {
  spi_t *spi = netif->state;

  uint16_t frames = ksz8851_reg_read(spi, ETH_RXFCTR) >> 8;

  for (; frames; frames--) {
    uint16_t status = ksz8851_reg_read(spi, ETH_RXFHSR);
    bool valid      = (status & RxFrameValid) &&
                 !(status & (RxCrcError | RxRuntFrame | RxFrameTooLong | RxMiiError | RxUdpFrameChecksumStatus |
                             RxTcpFrameChecksumStatus | RxIpFrameChecksumStatus | RxIcmpFrameChecksumStatus));
    if (!valid) {
#ifdef DEBUG
      putstr("KSZ8851: Invalid frame, status = ");
      puthexn(status, 4);
      puts("");
#endif

      ksz8851_drop_error_frame(spi);
      continue;
    }

    uint16_t len = ksz8851_reg_read(spi, ETH_RXFHBCR) & 0xFFF;
    if (len == 0) {
#ifdef DEBUG
      puts("KSZ8851: Zero length frame");
#endif
      ksz8851_drop_error_frame(spi);
      continue;
    }

#ifdef DEBUG
    putstr("KSZ8851: Receiving frame ");
    puthexn(status, 4);
    putchar(' ');
    puthexn(len, 4);
    puts("");
#endif

    struct pbuf *buf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    // Reset QMU RXQ frame pointer to zero.
    ksz8851_reg_write(spi, ETH_RXFDPR, 0x4000);

    // Start QMU DMA transfer operation
    ksz8851_reg_set(spi, ETH_RXQCR, StartDmaAccess);

    // Start receiving.
    uint8_t cmd = 0b10 << 6;
    spi_set_cs(spi, EthCsLine, 0);
    spi_tx(spi, &cmd, 1);

    uint8_t dummy[8];
    // Read the dummy initial word and headers (which we have already know from RXFHSR and RXFHBCR).
    spi_rx(spi, dummy, 8);

    uint16_t rem_len = len;
    for (struct pbuf *p = buf; rem_len != 0 && p != NULL; p = p->next) {
      uint32_t frag_len = p->len > rem_len ? rem_len : p->len;
      spi_rx(spi, p->payload, frag_len);
      rem_len -= frag_len;
    }

    // The receiving needs to be dword-aligned, so we read the dummy paddings.
    uint32_t pad = (-len) & 0x3;
    if (pad != 0) {
      spi_rx(spi, dummy, pad);
    }

    spi_set_cs(spi, EthCsLine, 1);

    // Stop QMU DMA transfer operation
    ksz8851_reg_clear(spi, ETH_RXQCR, StartDmaAccess);

#ifdef DEBUG
    putstr("KSZ8851: Received frame, len = ");
    puthexn(len, 4);
    puts("");
#endif

    netif->input(buf, netif);
  }
}

err_t ksz8851_poll(struct netif *netif) {
  spi_t *spi = netif->state;

  // Disable IRQ.
  uint32_t flags = arch_local_irq_save();

  uint32_t isr = ksz8851_reg_read(spi, ETH_ISR);
  if (isr & (1 << 13)) {
    // Acknowledging the interrupt.
    ksz8851_reg_write(spi, ETH_ISR, 1 << 13);

    ksz8851_recv(netif);
  }

  arch_local_irq_restore(flags);
}

static void ksz8851_irq_handler(irq_t irq) {
  spi_t *spi   = eth_netif->state;
  uint32_t isr = ksz8851_reg_read(spi, ETH_ISR);
  if (!isr) {
    return;
  }

  // Acknowledging the interrupts.
  ksz8851_reg_write(spi, ETH_ISR, isr);

  if (isr & (1 << 13)) {
    ksz8851_recv(eth_netif);
  }
}

err_t ksz8851_init(struct netif *netif) {
  spi_t *spi = netif->state;
  if (!spi) return ERR_ARG;

  // Reset chip
  set_output_bit(GPIO_OUT, EthRstPin, 0);
  timer_delay(150);
  set_output_bit(GPIO_OUT, EthRstPin, 0x1);

  uint16_t cider = ksz8851_reg_read(spi, ETH_CIDER);
  putstr("KSZ8851: Chip ID is ");
  puthexn(cider, 4);
  puts("");

  // Check the chip ID. The last nibble is revision ID and can be ignored.
  if ((cider & 0xFFF0) != 0x8870) {
    puts("KSZ8851: Unexpected Chip ID");
    return ERR_ARG;
  }

  // Write the MAC address and initialize MAC address in netif.
  struct eth_addr addr = ETH_ADDR(0x3a, 0x30, 0x25, 0x24, 0xfe, 0x7a);
  ksz8851_write_mac(spi, addr);
  putstr("KSZ8851: MAC address is ");
  for (int i = 0; i < 6; i++) {
    if (i != 0) putchar(':');
    puthexn(addr.addr[i], 2);
  }
  puts("");

  memcpy(netif->hwaddr, &addr, ETH_HWADDR_LEN);
  netif->hwaddr_len = ETH_HWADDR_LEN;

  // Enable QMU Transmit Frame Data Pointer Auto Increment
  ksz8851_reg_write(spi, ETH_TXFDPR, 0x4000);
  // Enable QMU Transmit flow control / Transmit padding / Transmit CRC, and IP/TCP/UDP checksum generation.
  ksz8851_reg_write(spi, ETH_TXCR, 0x00EE);
  // Enable QMU Receive Frame Data Pointer Auto Increment.
  ksz8851_reg_write(spi, ETH_RXFDPR, 0x4000);
  // Configure Receive Frame Threshold for one frame.
  ksz8851_reg_write(spi, ETH_RXFCTR, 0x0001);
  // Enable QMU Receive flow control / Receive all broadcast frames /Receive unicast frames, and IP/TCP/UDP checksum
  // verification etc.
  ksz8851_reg_write(spi, ETH_RXCR1, 0x7CE0);
  // Enable QMU Receive UDP Lite frame checksum verification, UDP Lite frame checksum generation, IPv6 UDP fragment
  // frame pass, and IPv4/IPv6 UDP UDP checksum field is zero pass.
  // In addition (not in the programmer's guide), enable single-frame data burst.
  ksz8851_reg_write(spi, ETH_RXCR2, 0x009C);
  // Enable QMU Receive Frame Count Threshold / RXQ Auto-Dequeue frame.
  ksz8851_reg_write(spi, ETH_RXQCR, RxFrameCountThresholdEnable | AutoDequeueRxQFrameEnable);

  // Programmer's guide have a step to set the chip in half-duplex when negotiation failed, but we omit the step.

  // Restart Port 1 auto-negotiation
  ksz8851_reg_set(spi, ETH_P1CR, 1 << 13);

  // Configure Low Watermark to 6KByte available buffer space out of 12KByte.
  ksz8851_reg_write(spi, ETH_FCLWR, 0x0600);
  // Configure High Watermark to 4KByte available buffer space out of 12KByte.
  ksz8851_reg_write(spi, ETH_FCHWR, 0x0400);

  // Clear the interrupt status
  ksz8851_reg_write(spi, ETH_ISR, 0xFFFF);
  // Enable Link Change/Transmit/Receive interrupt
  ksz8851_reg_write(spi, ETH_IER, 0xE000);
  // Enable QMU Transmit.
  ksz8851_reg_set(spi, ETH_TXCR, 1 << 0);
  // Enable QMU Receive.
  ksz8851_reg_set(spi, ETH_RXCR1, 1 << 0);

  timer_delay(1000);

  ksz8851_dump(spi);

  netif->linkoutput = ksz8851_output;
  netif->output     = etharp_output;
  netif->mtu        = 1500;

  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  // Initialize IRQ
  eth_netif = netif;
  rv_plic_register_irq(EthIntrIrq, ksz8851_irq_handler);
  rv_plic_enable(EthIntrIrq);

  puts("KSZ8851: Initialized");
  return ERR_OK;
}
