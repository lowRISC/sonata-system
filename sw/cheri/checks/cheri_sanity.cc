#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <cheri.hh>
#include <stdint.h>

using namespace CHERI;

#define GPIO_VALUE (0x00000FF0)

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot)
{
	Capability<void> root{rwRoot};

	// Capability to general purpose output
	Capability<volatile uint32_t> gpo = root.cast<volatile uint32_t>();
	gpo.address() = 0x80000000;
	gpo.bounds() = sizeof(uint32_t);

	// Capability to general purpose input
	Capability<volatile uint32_t> gpi = root.cast<volatile uint32_t>();
	gpi.address() = 0x80000004;
	gpi.bounds() = sizeof(uint32_t);

	// Use pointer to flash LEDs
	uint32_t gpioValue = 0;
	uint32_t inputValue = 0;
	uint32_t joystickValue = 0;
	uint32_t switchValue = 0;
	while (true) {
		gpioValue ^= GPIO_VALUE;
		for (int i = 0; i < 2000000; i++) {
			inputValue = *((volatile uint32_t *) gpi);
			// Shift right to remove joystick, mask to only get 8 switches and shift left to skip LCD controls.
			switchValue = ((inputValue >> 5) & 0xFF) << 4;
			joystickValue = (inputValue & 0x1F) << 4;
			*((volatile uint32_t *) gpo) = (gpioValue ^ switchValue ^ joystickValue) & GPIO_VALUE;
		}
	}
}
