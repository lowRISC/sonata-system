// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include "test_runner.hh"
#include "uart_tests.hh"
#include "hyperram_tests.hh"
#include "../common/sonata-peripherals.hh"
#include <cheri.hh>
#include <platform-uart.hh>
#include "../common/ostream.hh"

extern "C" void entry_point(void *rwRoot)
{
	CapRoot root{rwRoot};

	auto uart0 = uart_ptr(root);

	uart0->init(BAUD_RATE);
  auto console = LOG::OStream(uart0);
  
	uart_tests(root, console);
	hyperram_tests(root, console);
	finish_running(console, "All tests finished");
}
