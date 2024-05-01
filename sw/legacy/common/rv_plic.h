// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef RV_PLIC_H__
#define RV_PLIC_H__

#include "stdbool.h"
#include "stdint.h"

typedef uint32_t irq_t;
typedef void (*irq_handler_t)(irq_t);

void rv_plic_init();
void rv_plic_register_irq(irq_t, irq_handler_t);
bool rv_plic_pending(irq_t);
void rv_plic_enable(irq_t);
void rv_plic_disable(irq_t);

#endif  // RV_PLIC_H__
