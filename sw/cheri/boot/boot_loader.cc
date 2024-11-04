/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

#include <algorithm>
#include "../../common/defs.h"
#include <cheri.hh>
#include <platform-gpio.hh>
#include <platform-spi.hh>
#include <platform-uart.hh>

#include "../common/asm.hh"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"
#include "elf.h"

extern "C" {
// Use a different name to avoid resolve to CHERIoT-RTOS memset symbol.
void bl_memset(void *, int, size_t);
}

#define DEBUG_ELF_HEADER 0

#define ARR_LEN(X) ((sizeof(X)) / (sizeof(X[0])))

const uint32_t SoftwareSlots[] = {
    0 * 10 * 1024 * 1024,  // Slot 1
    1 * 10 * 1024 * 1024,  // Slot 2
    2 * 10 * 1024 * 1024,  // Slot 3
};

const uint8_t SoftwareSelectGpioPins[] = {13, 14, 15};

const char prefix[] = "\x1b[35mbootloader\033[0m: ";

typedef CHERI::Capability<volatile OpenTitanUart> &UartRef;
typedef CHERI::Capability<volatile SonataGPIO> &GpioRef;

[[noreturn]] void complain_and_loop(UartRef uart, const char *str) {
  write_str(uart, prefix);
  write_str(uart, str);
  while (true) {
    asm("wfi");
  }
}

uint8_t read_selected_software_slot(GpioRef gpio) {
  for (uint8_t i = 0; i < ARR_LEN(SoftwareSelectGpioPins); i++) {
    if (gpio->input & (1 << SoftwareSelectGpioPins[i])) {
      return i;
    }
  }
  return 0;  // Default to software slot 1 (i.e. SW0)
}

static void debug_print_phdr(UartRef uart, Elf32_Phdr &phdr) {
  write_str(uart, prefix);
  write_hex(uart, phdr.p_offset);
  write_str(uart, " ");
  write_hex(uart, phdr.p_vaddr);
  write_str(uart, " ");
  write_hex(uart, phdr.p_filesz);
  write_str(uart, " ");
  write_hex(uart, phdr.p_memsz);
  write_str(uart, "\r\n");
}

static void write_hex_with_prefix(UartRef uart, const char *msg, uint32_t value) {
  write_str(uart, prefix);
  write_str(uart, msg);
  write_hex(uart, value);
  write_str(uart, "\r\n");
}

uint32_t read_elf(SpiFlash &flash, uint32_t addr, UartRef uart, CHERI::Capability<uint8_t> sram,
                  CHERI::Capability<uint8_t> hyperram) {
  write_str(uart, prefix);
  write_str(uart, "Loading software from flash...\r\n");

  Elf32_Ehdr ehdr;
  flash.read(addr, (uint8_t *)&ehdr, sizeof(Elf32_Ehdr));

  // Check the ELF magic numbers.
  if (ehdr.e_ident[EI_MAG0] != ELFMAG0 || ehdr.e_ident[EI_MAG1] != ELFMAG1 || ehdr.e_ident[EI_MAG2] != ELFMAG2 ||
      ehdr.e_ident[EI_MAG3] != ELFMAG3) {
    complain_and_loop(uart, "Failed ELF Magic Check\r\n");
  }

  if (ehdr.e_ident[EI_CLASS] != ELFCLASS32 || ehdr.e_type != ET_EXEC || ehdr.e_machine != EM_RISCV ||
      (ehdr.e_flags & (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE)) != (EF_RISCV_CHERIABI | EF_RISCV_CAP_MODE)) {
    complain_and_loop(uart, "ELF file is not 32-bit CHERI RISC-V executable\r\n");
  }

#if DEBUG_ELF_HEADER
  write_str(uart, prefix);
  write_str(uart, "Offset   VirtAddr FileSize MemSize\r\n");
#endif

  Elf32_Phdr phdr;
  for (uint32_t i = 0; i < ehdr.e_phnum; i++) {
    uint32_t phdr_read_addr = addr + ehdr.e_phoff;
    flash.read(phdr_read_addr + ehdr.e_phentsize * i, (uint8_t *)&phdr, sizeof(Elf32_Phdr));

    if (phdr.p_type != PT_LOAD) continue;

#if DEBUG_ELF_HEADER
    debug_print_phdr(uart, phdr);
#endif

    auto segment      = phdr.p_vaddr >= sram.top() ? hyperram : sram;
    segment.address() = phdr.p_vaddr;
    segment.bounds().set_inexact(phdr.p_memsz);

    if (!segment.is_valid()) {
      debug_print_phdr(uart, phdr);
      complain_and_loop(uart, "Cannot get a valid capability for segment\n");
    }

    for (uint32_t offset = 0; offset < phdr.p_filesz; offset += 0x400) {
      uint32_t size = std::min(phdr.p_filesz, offset + 0x400) - offset;
      flash.read(addr + phdr.p_offset + offset, segment.get() + offset, size);
    }

    bl_memset(segment.get() + phdr.p_filesz, 0, phdr.p_memsz - phdr.p_filesz);
  }

  return ehdr.e_entry;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t rom_loader_entry(void *rwRoot) {
  CHERI::Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                                 = UART_ADDRESS;
  uart.bounds()                                  = UART_BOUNDS;

  CHERI::Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                             = SPI_ADDRESS;
  spi.bounds()                              = SPI_BOUNDS;

  CHERI::Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                              = GPIO_ADDRESS;
  gpio.bounds()                               = GPIO_BOUNDS;

  CHERI::Capability<uint8_t> sram = root.cast<uint8_t>();
  sram.address()                  = SRAM_ADDRESS + 0x1000;
  sram.bounds()                   = SRAM_BOUNDS - 0x1000;

  CHERI::Capability<uint8_t> hyperram = root.cast<uint8_t>();
  hyperram.address()                  = HYPERRAM_ADDRESS;
  hyperram.bounds()                   = HYPERRAM_BOUNDS;

  CHERI::Capability<uint32_t> sysinfo = root.cast<uint32_t>();
  sysinfo.address()                   = SYSTEM_INFO_ADDRESS;
  sysinfo.bounds()                    = SYSTEM_INFO_BOUNDS;

  uint32_t git_hash_0 = sysinfo[0];
  uint32_t git_hash_1 = sysinfo[1];
  uint32_t git_dirty  = sysinfo[2];

  spi->init(false, false, true, 0);
  uart->init(BAUD_RATE);

  SpiFlash spi_flash(spi);
  spi_flash.reset();

  write_str(uart, prefix);
  write_str(uart, "Sonata system git SHA: ");
  write_hex(uart, git_hash_0);
  write_hex(uart, git_hash_1);
  if (git_dirty) {
    write_str(uart, " (dirty)");
  }
  write_str(uart, "\r\n");

  uint8_t software_slot = read_selected_software_slot(gpio);
  write_str(uart, prefix);
  write_str(uart, "Selected software slot: ");
  char software_slot_str[] = "1\r\n";
  software_slot_str[0] += software_slot;
  write_str(uart, software_slot_str);
  uint32_t flash_addr = SoftwareSlots[software_slot];

  uint32_t entrypoint = read_elf(spi_flash, flash_addr, uart, sram, hyperram);

  write_str(uart, prefix);
  write_str(uart, "Booting into program, hopefully.\r\n");
  return entrypoint;
}

extern "C" void exception_handler(void *rwRoot) {
  CHERI::Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  CHERI::Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                                 = UART_ADDRESS;
  uart.bounds()                                  = UART_BOUNDS;

  write_str(uart, prefix);
  write_str(uart, "Exception happened during loading.\r\n");

  write_hex_with_prefix(uart, "mepc  : ", READ_CSR("mepc"));
  write_hex_with_prefix(uart, "mcause: ", READ_CSR("mcause"));
  write_hex_with_prefix(uart, "mtval : ", READ_CSR("mtval"));
}
