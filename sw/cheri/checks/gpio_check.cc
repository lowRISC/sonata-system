#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include "../../common/defs.h"
#include <cheri.hh>
#include <platform-gpio.hh>
#include <stdint.h>

using namespace CHERI;

static constexpr uint32_t wait_cycles = (CPU_TIMER_HZ / 10);

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot)
{
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address() = GPIO_ADDRESS;
  gpio.bounds()  = GPIO_BOUNDS;

  int count = 0;
  bool switch_on = true;
  while (true) {
    for (int i = 0; i < wait_cycles; ++i) {
      asm("");
    }
    if (switch_on) {
      gpio->led_on(count);
    } else {
      gpio->led_off(count);
    };
    switch_on = (count == 7) ? !switch_on : switch_on;
    count = (count < 7) ? count + 1 : 0;
  }
}
