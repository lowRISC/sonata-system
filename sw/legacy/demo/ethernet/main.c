// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <lwip/timeouts.h>
#include <netif/ethernet.h>

#include "ksz8851.h"
#include "sonata_system.h"
#include "spi.h"
#include "timer.h"

enum {
  // GPIO Input
  EthIntrPin = 13,

  // GPIO Output
  EthCsPin  = 13,
  EthRstPin = 14,
};

void eth_callback(struct netif* netif, netif_nsc_reason_t reason, const netif_ext_callback_args_t* args) {
  if (reason & LWIP_NSC_IPV4_ADDR_VALID) {
    putstr("IPv4 address available: ");
    ip4_addr_t *addr = ip_2_ip4(&netif->ip_addr);
    putdec(ip4_addr1(addr));
    putchar('.');
    putdec(ip4_addr2(addr));
    putchar('.');
    putdec(ip4_addr3(addr));
    putchar('.');
    putdec(ip4_addr4(addr));
    puts("");
  }
}

int main(void) {
  uart_init(DEFAULT_UART);
  puts("Ethernet demo application");

  timer_init();

  lwip_init();

  spi_t spi;
  spi_init(&spi, ETH_SPI, 0 /* speed, currently unused */);

  struct netif netif;
  netif_add(&netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, &spi, ksz8851_init, ethernet_input);
  netif.name[0] = 'e';
  netif.name[1] = '0';
  netif_set_default(&netif);
  netif_set_up(&netif);

  netif_set_link_up(&netif);

  dhcp_start(&netif);

  netif_ext_callback_t callback;
  netif_add_ext_callback(&callback, eth_callback);

  while (1) {
    ksz8851_poll(&netif);
    sys_check_timeouts();
  }

  return 0;
}
