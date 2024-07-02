#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"
#include "elf.h"

#include <algorithm>
#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-spi.hh>
#include <platform-uart.hh>
#include <stdint.h>

const char prefix[] = "\x1b[35mbootloader\033[0m: ";

typedef CHERI::Capability<volatile OpenTitanUart<>> &UartRef;

[[noreturn]] void complain_and_loop(UartRef uart, const char *str)
{
	write_str(uart, prefix);
	write_str(uart, str);
	while (true)
	{
		asm("wfi");
	}
}

void read_elf(SpiFlash &flash, UartRef uart, CHERI::Capability<uint32_t> sram)
{
	write_str(uart, prefix);
	write_str(uart, "Loading software from flash...\r\n");

	Elf32_Ehdr ehdr;
	flash.read(0x0, (uint8_t *)&ehdr, sizeof(Elf32_Ehdr));

	// Check the ELF magic numbers.
	if (ehdr.e_ident[EI_MAG0] != ELFMAG0 || ehdr.e_ident[EI_MAG1] != ELFMAG1 ||
	    ehdr.e_ident[EI_MAG2] != ELFMAG2 || ehdr.e_ident[EI_MAG3] != ELFMAG3)
	{
		complain_and_loop(uart, "Failed ELF Magic Check\r\n");
	}

	if (ehdr.e_ident[EI_CLASS] != ELFCLASS32 || ehdr.e_type != ET_EXEC ||
	    ehdr.e_machine != EM_RISCV ||
	    (ehdr.e_flags & (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE)) !=
	      (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE))
	{
		complain_and_loop(uart,
		                  "ELF file is not 32-bit CHERI RISC-V executable\r\n");
	}

	write_str(uart, prefix);
	write_str(uart, "Offset   VirtAddr FileSize MemSize\r\n");

	Elf32_Phdr phdr;
	for (uint32_t i = 0; i < ehdr.e_phnum; i++)
	{
		flash.read(ehdr.e_phoff + ehdr.e_phentsize * i,
		           (uint8_t *)&phdr,
		           sizeof(Elf32_Phdr));

		if (phdr.p_type != PT_LOAD)
			continue;

		write_str(uart, prefix);
		write_hex(uart, phdr.p_offset);
		write_str(uart, " ");
		write_hex(uart, phdr.p_vaddr);
		write_str(uart, " ");
		write_hex(uart, phdr.p_filesz);
		write_str(uart, " ");
		write_hex(uart, phdr.p_memsz);
		write_str(uart, "\r\n");

		auto segment      = sram;
		segment.address() = phdr.p_vaddr;
		segment.bounds().set_inexact(phdr.p_memsz);

		for (uint32_t offset = 0; offset < phdr.p_filesz; offset += 0x400)
		{
			uint32_t size = std::min(phdr.p_filesz, offset + 0x400) - offset;
			flash.read(
			  phdr.p_offset + offset, (uint8_t *)segment.get() + offset, size);
		}

		// We don't have memset symbol available.
		for (uint32_t offset = phdr.p_filesz; offset < phdr.p_memsz; offset++)
		{
			((uint8_t *)segment.get())[offset] = 0;
		}
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
	read_elf(spi_flash, uart, sram);

	write_str(uart, prefix);
	write_str(uart, "Booting into program, hopefully.\r\n");
}
