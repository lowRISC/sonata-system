#pragma once
#include "../../common/defs.h"
#include <platform-uart.hh>

typedef volatile OpenTitanUart<> *UartPtr;

static void write_str(UartPtr uart, const char *str)
{
	for (; *str != '\0'; ++str)
	{
		uart->blocking_write(*str);
	}
}

template<uint32_t Digits>
static void to_hex(char str_buf[Digits + 1], uint32_t num)
{
	static_assert(Digits < 9);
	for (size_t i = 0; i < Digits; ++i)
	{
		if ((num & 0xf) < 10)
		{
			str_buf[Digits - 1 - i] = (num & 0xf) + '0';
		}
		else
		{
			str_buf[Digits - 1 - i] = (num & 0xf) + 'a' - 10;
		}
		num >>= 4;
	}
	str_buf[Digits] = 0;
}

[[maybe_unused]] static void write_hex(volatile OpenTitanUart<> *uart,
                                       uint32_t                  num)
{
	char str_buf[9];
	to_hex<8>(str_buf, num);
	write_str(uart, str_buf);
}

static void write_hex8b(volatile OpenTitanUart<> *uart, uint8_t num)
{
	char str_buf[3];
	to_hex<2>(str_buf, num);
	write_str(uart, str_buf);
}
