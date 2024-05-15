#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"

#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-spi.hh>
#include <platform-uart.hh>
#include <stdint.h>

const char prefix[] = "\x1b[35mbootloader\033[0m: ";

struct UF2Block
{
	uint64_t magicStart;
	uint32_t flags;
	uint32_t targetAddr;
	uint32_t payloadSize;
	uint32_t blockNo;
	uint32_t numBlocks;
	uint32_t fileSizeOrFamilyId;
	uint32_t data[119];
	uint32_t magicEnd;

	static constexpr size_t size = 512;

	void read_from_flash(SpiFlash &flash, uint32_t address)
	{
		flash.read(address, (uint8_t *)this, size);
	}

	auto check_magic() -> bool
	{
		return 0x9e5d'5157'0a32'4655 == magicStart && 0x0ab1'6f30 == magicEnd;
	}
};

static_assert(sizeof(UF2Block) == UF2Block::size,
              "The UF2Block structure is the wrong size.");

typedef CHERI::Capability<volatile OpenTitanUart<>> &UartRef;

[[noreturn]] void complain_and_loop(UartRef uart, const char *str)
{
	write_str(uart, prefix);
	write_str(uart, str);
	while (true) {
		asm ("wfi");
	}
}

void read_blocks(SpiFlash                   &flash,
                 UartRef                     uart,
                 CHERI::Capability<uint32_t> sram)
{
	write_str(uart, prefix);
	write_str(uart, "Loading software from flash...\r\n");

	UF2Block block;
	block.read_from_flash(flash, 0x0);

	if (!block.check_magic())
	{
		complain_and_loop(uart, "Failed Magic Check\r\n");
	}
	auto write_block = [sram](UF2Block &block) {

		auto block_addrs      = sram;
		block_addrs.address() = block.targetAddr;
		block_addrs.bounds()  = block.payloadSize;
		const size_t words    = block.payloadSize / 4;

		for (uint32_t word = 0; word < words; ++word)
		{
			block_addrs[word] = block.data[word];
		}
	};
	write_block(block);

	const uint32_t num_blocks   = block.numBlocks;
	uint32_t       next_address = 0x0 + UF2Block::size;

	for (size_t i = 1; i < num_blocks; ++i)
	{
		block.read_from_flash(flash, next_address);
		if (!block.check_magic() || num_blocks != block.numBlocks ||
		    i != block.blockNo)
		{
			complain_and_loop(uart, "Failed Magic or numBlocks Check\r\n");
		}
		write_block(block);
		next_address += UF2Block::size;
	}
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" void rom_loader_entry(void *rwRoot)
{
	CHERI::Capability<void> root{rwRoot};

	// Create a bounded capability to the UART
	CHERI::Capability<volatile OpenTitanUart<>> uart =
	  root.cast<volatile OpenTitanUart<>>();
	uart.address() = UART_ADDRESS;
	uart.bounds()  = UART_BOUNDS;

	CHERI::Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
	spi.address()                             = SPI_ADDRESS;
	spi.bounds()                              = SPI_BOUNDS;

	CHERI::Capability<volatile SonataGPIO> gpio =
	  root.cast<volatile SonataGPIO>();
	gpio.address() = GPIO_ADDRESS;
	gpio.bounds()  = GPIO_BOUNDS;

	CHERI::Capability<uint32_t> sram = root.cast<uint32_t>();
	sram.address()                   = 0x00101000;
	sram.bounds()                    = 0x00040000 - 0x1000;

	spi->init(false, false, true, 0);
	uart->init();

	SpiFlash spi_flash(spi, gpio, FLASH_CSN_GPIO_BIT);
	read_blocks(spi_flash, uart, sram);

	write_str(uart, prefix);
	write_str(uart, "Booting into program, hopefully.\r\n");
}
