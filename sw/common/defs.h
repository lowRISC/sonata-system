/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CPU_TIMER_HZ (40'000'000)
#define BAUD_RATE (921'600)

#define SRAM_ADDRESS (0x0010'0000)
#define SRAM_BOUNDS (0x0002'0000)

#define HYPERRAM_ADDRESS (0x4000'0000)
#define HYPERRAM_BOUNDS (0x0080'0000)
// The portion of the HyperRAM that can support capabilities.
#define HYPERRAM_TAG_BOUNDS (0x0040'0000)

#define SYSTEM_INFO_ADDRESS (0x8000'C000)
#define SYSTEM_INFO_BOUNDS (0x0000'0034)

#define GPIO_NUM 6
#define GPIO_ADDRESS (0x8000'0000)
#define GPIO_BOUNDS (0x0000'001C)
#define GPIO_RANGE (0x0000'0040)

#define PWM_NUM 7
#define PWM_ADDRESS (0x8000'1000)
#define PWM_BOUNDS (0x0000'0008)
#define PWM_RANGE (0x0000'0008)

// The PWM used as the LCD backlight is the last PWM output.
#define PWM_LCD (PWM_NUM - 1)

#define RGBLED_CTRL_ADDRESS (0x8000'9000)
#define RGBLED_CTRL_BOUNDS (0x0000'0010)

#define REVOKER_ADDRESS (0x8000'A000)
#define REVOKER_BOUNDS (0x0000'1000)

#define ADC_ADDRESS (0x8000'B000)
#define ADC_BOUNDS (0x0000'1000)

#define CLINT_ADDRESS (0x8004'0000)
#define CLINT_BOUNDS (0x0001'0000)

#define UART_NUM 3
#define UART_ADDRESS (0x8010'0000)
#define UART_BOUNDS (0x0000'0034)
#define UART_RANGE (0x0000'1000)

#define I2C_NUM 2
#define I2C_ADDRESS (0x8020'0000)
#define I2C_BOUNDS (0x0000'0080)
#define I2C_RANGE (0x0000'1000)

#define SPI_NUM 5
#define SPI_ADDRESS (0x8030'0000)
#define SPI_BOUNDS (0x0000'002C)
#define SPI_RANGE (0x0000'1000)

#define USBDEV_ADDRESS (0x8040'0000)
#define USBDEV_BOUNDS (0x0000'1000)

#define PINMUX_PIN_SINKS_ADDRESS (0x8000'5000)
#define PINMUX_PIN_SINKS_BOUNDS (0x0000'0055)
#define PINMUX_BLOCK_SINKS_ADDRESS (0x8000'5800)
#define PINMUX_BLOCK_SINKS_BOUNDS (0x0000'0046)

#define PLIC_ADDRESS (0x8800'0000)
#define PLIC_BOUNDS (0x0040'0000)
