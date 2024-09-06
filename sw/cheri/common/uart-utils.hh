/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "../../common/defs.h"
#include <platform-uart.hh>
#include "../common/ostream.hh"

typedef volatile OpenTitanUart *UartPtr;

static void write_str(UartPtr uart, const char *str)
{
	for (; *str != '\0'; ++str)
	{
		uart->blocking_write(*str);
	}
}

[[maybe_unused]] static void write_hex(volatile OpenTitanUart *uart,
                                       uint32_t                  num)
{
  std::array<char,9> str_buf;
	write_str(uart, LOG::to_hex_str(str_buf, num));
}

[[maybe_unused]] static void write_hex8b(volatile OpenTitanUart *uart,
                                         uint8_t                   num)
{
  std::array<char,3> str_buf;
	write_str(uart, LOG::to_hex_str(str_buf, num));
}
