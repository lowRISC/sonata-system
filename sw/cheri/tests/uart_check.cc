#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <cheri.hh>
#include <platform-uart.hh>
#include <stdint.h>

using namespace CHERI;

#define GPIO_VALUE  (0xFFFFFFFF)
#define CPU_FREQ_HZ (30'000'000)
#define BAUD_RATE   (   115'200)

/**
 * OpenTitan UART
 */
class OpenTitanUart
{
  typedef uint32_t RegisterType;

 [[maybe_unused]] RegisterType intrState;
 [[maybe_unused]] RegisterType intrEnable;
 [[maybe_unused]] RegisterType intrTest;
 [[maybe_unused]] RegisterType alertTest;
 [[maybe_unused]] RegisterType ctrl;
 [[maybe_unused]] RegisterType status;
 [[maybe_unused]] RegisterType rData;
 RegisterType wData;
 [[maybe_unused]] RegisterType fifoCtrl;
 RegisterType fifoStatus;
 [[maybe_unused]] RegisterType ovrd;
 [[maybe_unused]] RegisterType val;
 [[maybe_unused]] RegisterType timeoutCtrl;

  public:
  void init() volatile
  {
    // NCO = 2^20 * baud rate / cpu frequency
    const uint32_t nco = (((uint64_t) BAUD_RATE << 20) / CPU_FREQ_HZ);
    // Set the baud rate and enable transmit & receive
    ctrl = (nco << 16) | 0b11;
  };

  bool can_write() volatile
  {
    return (fifoStatus & 0xff) < 32;
  };

  /**
   * Write one byte, blocking until the byte is written.
   */
  void blocking_write(uint8_t byte) volatile
  {
    while (!can_write()) {}
    wData = byte;
  }
};


/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]]
extern "C" void rom_loader_entry(void *rwRoot)
{
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address() = 0x80100000;
  uart.bounds()  = 0x34;

  uart->init();
  uart->blocking_write('h');
  uart->blocking_write('i');
  uart->blocking_write('\r');
  uart->blocking_write('\n');
  while (true) {
    uart->blocking_write('h');
    uart->blocking_write('i');
    uart->blocking_write('\r');
    uart->blocking_write('\n');
  }
}
