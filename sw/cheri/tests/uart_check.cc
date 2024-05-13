#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include <cheri.hh>
#include <platform-uart.hh>
#include <stdint.h>

using namespace CHERI;

void write(volatile OpenTitanUart<>* uart, const char* str) {
  for (; *str != '\0'; ++str) {
    uart->blocking_write(*str);
  }
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]]
extern "C" void rom_loader_entry(void *rwRoot)
{
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart<>> uart = root.cast<volatile OpenTitanUart<>>();
  uart.address() = UART_ADDRESS;
  uart.bounds()  = UART_BOUNDS;

  uart->init(BAUD_RATE);
  write(uart, "Hello There!\r\n");

  char ch = '\n';
  while (true) {
    ch = uart->blocking_read();
    uart->blocking_write(ch);
  }
}
