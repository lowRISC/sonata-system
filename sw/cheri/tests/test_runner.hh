/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "../common/sonata-peripherals.hh"
#include "../common/uart-utils.hh"

[[noreturn]] static void finish_running(UartPtr uart, const char *message)
{
    write_str(uart, message);
    write_str(uart, "\r\n");

    while(true) asm volatile ("wfi");
}

[[maybe_unused]] static void check_result(UartPtr console, bool result)
{
	if (result)
	{
		return;
	}
	finish_running(console, "Test(s) Failed");
}
