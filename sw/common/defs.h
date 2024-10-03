/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CPU_TIMER_HZ (30'000'000)
#define BAUD_RATE (921'600)

#define SRAM_ADDRESS (0x0010'0000)
#define SRAM_BOUNDS (0x0002'0000)

#define RGBLED_CTRL_ADDRESS (0x8000'9000)
#define RGBLED_CTRL_BOUNDS (0x0000'0010)

#define GPIO_ADDRESS (0x8000'0000)
#define GPIO_BOUNDS (0x0000'0020)

#define UART_BOUNDS (0x0000'0034)
#define UART_ADDRESS (0x8010'0000)
#define UART1_ADDRESS (0x8010'1000)

#define I2C_BOUNDS (0x0000'0080)
#define I2C_ADDRESS (0x8020'0000)
#define I2C1_ADDRESS (0x8020'1000)

#define SPI_BOUNDS (0x0000'002C)
#define SPI_ADDRESS (0x8030'0000)
#define SPI1_ADDRESS (0x8030'1000)
#define SPI2_ADDRESS (0x8030'2000)

#define USBDEV_ADDRESS (0x8040'0000)
#define USBDEV_BOUNDS (0x0000'1000)

#define PLIC_ADDRESS (0x8800'0000)
#define PLIC_BOUNDS (0x0040'0000)

#define HYPERRAM_ADDRESS (0x4000'0000)
#define HYPERRAM_BOUNDS (0x0010'0000)
