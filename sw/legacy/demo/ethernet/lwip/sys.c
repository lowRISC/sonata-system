// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <lwip/opt.h>
#include <sonata_system.h>
#include <timer.h>

static int protection_depth = 0;

sys_prot_t sys_arch_protect(void) { return arch_local_irq_save(); }

void sys_arch_unprotect(sys_prot_t val) { arch_local_irq_restore(val); }

u32_t sys_now(void) { return timer_read() / (SYSCLK_FREQ / 1000); }
