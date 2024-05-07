// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef LWIP_LWIPOPTS_H
#define LWIP_LWIPOPTS_H

#include <stdint.h>

typedef uint32_t sys_prot_t;

#define LWIP_SOCKET 0
#define LWIP_NETCONN 0
#define LWIP_DHCP 1
#define LWIP_NETIF_EXT_STATUS_CALLBACK 1

#endif
