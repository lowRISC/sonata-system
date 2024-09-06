/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "../common/sonata-peripherals.hh"
#include "../common/ostream.hh"

[[noreturn]] static void finish_running(LOG::OStream& console, const char *message)
{
    console <<  message << LOG::endl;

    while(true) asm volatile ("wfi");
}

[[maybe_unused]] static void check_result(LOG::OStream& console, bool result)
{
	if (result)
	{
		return;
	}
	finish_running(console, "Test(s) Failed");
}
