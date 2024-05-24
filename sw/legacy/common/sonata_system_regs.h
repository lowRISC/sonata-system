// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef SONATA_SYSTEM_REGS_H__
#define SONATA_SYSTEM_REGS_H__

#define GPIO_BASE  0x80000000

#define PWM_BASE   0x80001000

#define TIMER_BASE 0x80040000

#define UART0_BASE 0x80100000
#define UART1_BASE 0x80101000

#define I2C0_BASE  0x80200000
#define I2C1_BASE  0x80201000

#define SPI0_BASE  0x80300000
#define SPI1_BASE  0x80301000
#define SPI2_BASE  0x80302000
#define SPI3_BASE  0x80303000
#define SPI4_BASE  0x80304000
#define SPI5_BASE  0x80305000

#define USBDEV0_BASE 0x80400000

#define RV_PLIC_BASE 0x88000000

#define SIM_CTRL_BASE 0x20000
#define SIM_CTRL_OUT 0x0
#define SIM_CTRL_CTRL 0x8

#endif
