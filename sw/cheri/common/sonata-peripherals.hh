#pragma once
#include "../../common/defs.h"
#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-uart.hh>

typedef CHERI::Capability<void> CapRoot;
typedef volatile SonataGPIO *GpioPtr;
typedef volatile OpenTitanUart *UartPtr;

[[maybe_unused]] static GpioPtr gpio_ptr(CapRoot root) {
	CHERI::Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
	gpio.address() = GPIO_ADDRESS;
	gpio.bounds()  = GPIO_BOUNDS;
	return gpio;
}

[[maybe_unused]] static UartPtr uart_ptr(CapRoot root, uint32_t uart_idx = 0) {
	CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
	switch (uart_idx) {
	case 1:
		uart.address() = UART1_ADDRESS;
		break;
	default:
		uart.address() = UART_ADDRESS;
	};
	uart.bounds()  = UART_BOUNDS;
	return uart;
};
