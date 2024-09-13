// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef KSZ8851_H
#define KSZ8851_H

#include <lwip/netif.h>
#include <stdbool.h>

err_t ksz8851_init(struct netif *netif);
err_t ksz8851_poll(struct netif *netif);

#define ETH_MARL 0x10  // MAC address low
#define ETH_MARM 0x12  // MAC address middle
#define ETH_MARH 0x14  // MAC address high

#define ETH_GRR 0x26

#define ETH_TXCR 0x70     // Transmit control register
#define ETH_TXMIR 0x78    // TXQ memory information register
#define ETH_TXQCR 0x80    // TXQ command register
#define ETH_TXFDPR 0x84   // TX frame data pointer register
#define ETH_TXNTFSR 0x9E  // TX next total frames size register

#define ETH_RXCR1 0x74    // Receive control register 1
#define ETH_RXCR2 0x76    // Receive control register 2
#define ETH_RXFHSR 0x7c   // Receive frame header status register
#define ETH_RXFHBCR 0x7e  // Receive frame header byte count register
#define ETH_RXQCR 0x82    // RXQ control register
#define ETH_RXFDPR 0x86   // RX frame data pointer register
#define ETH_IER 0x90      // Interrupt enable register
#define ETH_ISR 0x92      // Interrupt status register
#define ETH_RXFCTR 0x9c   // RX frame count and threshold register

#define ETH_FCLWR 0xB0
#define ETH_FCHWR 0xB2

#define ETH_CIDER 0xc0    // Chip ID and enable register
#define ETH_P1MBCR 0xe4   // PHY 1 MII-register basic control register
#define ETH_P1MBSR 0xe6   // PHY 1 MII-register basic status register
#define ETH_P1SCLMD 0xf4  // Port 1 PHY special control/status, LinkMD
#define ETH_P1CR 0xf6     // Port 1 control register
#define ETH_P1SR 0xf8     // Port 1 status register

// Fields of RXFHSR
enum {
  RxCrcError                = 1 << 0,
  RxRuntFrame               = 1 << 1,
  RxFrameTooLong            = 1 << 2,
  RxFrameType               = 1 << 3,
  RxMiiError                = 1 << 4,
  RxUnicastFrame            = 1 << 5,
  RxMulticastFrame          = 1 << 6,
  RxBroadcastFrame          = 1 << 7,
  RxUdpFrameChecksumStatus  = 1 << 10,
  RxTcpFrameChecksumStatus  = 1 << 11,
  RxIpFrameChecksumStatus   = 1 << 12,
  RxIcmpFrameChecksumStatus = 1 << 13,
  RxFrameValid              = 1 << 15,
};

// Fields of RXQCR
enum {
  ReleaseRxErrorFrame            = 1 << 0,
  StartDmaAccess                 = 1 << 3,
  AutoDequeueRxQFrameEnable      = 1 << 4,
  RxFrameCountThresholdEnable    = 1 << 5,
  RxDataByteCountThresholdEnable = 1 << 6,
  RxDurationTimerThresholdEnable = 1 << 7,
  RxIpHeaderTwoByteOffsetEnable  = 1 << 9,
  RxFrameCountThresholdStatus    = 1 << 10,
  RxDataByteCountThresholdstatus = 1 << 11,
  RxDurationTimerThresholdStatus = 1 << 12,
};

// Fields of TXQCR
enum {
  ManualEnqueueTxQFrameEnable = 1 << 0,
  TxQMemoryAvailableMonitor   = 1 << 1,
  AutoEnqueueTxQFrameEnable   = 1 << 2,
};

#endif
