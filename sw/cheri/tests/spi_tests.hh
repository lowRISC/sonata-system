// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "../../common/defs.h"
#include "../common/console-utils.hh"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"
#include "test_runner.hh"
#include <cheri.hh>
#include <ds/xoroshiro.h>
#include <platform-uart.hh>

using namespace CHERI;

/**
 * Configures the number of test iterations to perform.
 * This can be overriden via a compilation flag.
 */
#ifndef SPI_TEST_ITERATIONS
#define SPI_TEST_ITERATIONS (1U)
#endif

/**
 * Configures the number of random sectors that will be erased in each
 * iteration of the SPI flash sector erasing test.
 * This can be overriden via a compilation flag.
 */
#ifndef SPI_TEST_FLASH_SECTORS_ERASED
#define SPI_TEST_FLASH_SECTORS_ERASED (1U)
#endif

/**
 * Configures the number of random pages that will be written with
 * random data in each iteration of the SPI flash random data test.
 * This can be overriden via a compilation flag.
 */
#ifndef SPI_TEST_FLASH_PAGES_WRITTEN
#define SPI_TEST_FLASH_PAGES_WRITTEN (10U)
#endif

// The expected JEDEC ID to read from the SPI Flash
static constexpr uint8_t ExpectedSpiFlashJedecId[3] = {0xEF, 0x40, 0x19};

/**
 * SPI Flash size definitions. All in bytes.
 * References to `sector` refer to the smallest erasable sector.
 *
 * Sourced from the datasheet:
 * https://www.winbond.com/resource-files/w25q256jv%20spi%20revg%2008032017.pdf
 */
static constexpr unsigned SpiFlashPageSize   = 256;               // 256 B
static constexpr unsigned SpiFlashSectorSize = 4 * 1024;          // 4 KiB
static constexpr unsigned SpiFlashSize       = 32 * 1024 * 1024;  // 32 MiB
static constexpr size_t SpiFlashPages        = SpiFlashSize / SpiFlashPageSize;
static constexpr size_t SpiFlashSectors      = SpiFlashSize / SpiFlashSectorSize;

/**
 * Get the start address of a random page in flash.
 */
inline uint32_t random_flash_page_addr(ds::xoroshiro::P32R8 &prng) {
  uint32_t page_num = (prng() << 16) | (prng() << 8) | prng();
  page_num %= SpiFlashPages;
  return page_num * SpiFlashPageSize;
}

/**
 * Get the start address of a random sector in flash.
 */
inline uint32_t random_flash_sector_addr(ds::xoroshiro::P32R8 &prng) {
  uint16_t sector_num = (prng() << 8) | prng();
  sector_num %= SpiFlashSectors;
  return sector_num * SpiFlashSectorSize;
}

/**
 * Erase a sector of flash, and then check read back the values from that
 * sector to check that they were properly erased. The size of this sector
 * is retrieved from the `SpiFlashSectorSize` definition. Assumes that
 * the SPI has already been appropriately configured. The address is
 * assumed to be sector-aligned.
 * Returns the number of failures during the test.
 */
static int spi_flash_erase_test_sector(Capability<volatile SonataSpi> spi, ds::xoroshiro::P32R8 &prng,
                                       SpiFlash spi_flash, uint32_t start_addr) {
  int failures = 0;
  std::array<uint8_t, SpiFlashPageSize> write_values, read_values;

  // Write a dummy value to the first page of the specified
  // sector in flash, so we can guarantee an erase is happening.
  constexpr uint8_t DummyWriteData = 0x3F;
  write_values.fill(DummyWriteData);
  spi_flash.write_page(start_addr, write_values.data());

  // Read the values we just wrote to verify that the data is there.
  spi_flash.read(start_addr, read_values.data(), SpiFlashPageSize);
  for (auto &read_value : read_values) {
    if (read_value != DummyWriteData) failures++;
  }

  // Erase the specified sector in flash
  spi_flash.erase_sector(start_addr);

  // Read erased sector, one page at a time.
  const uint32_t end_addr = start_addr + SpiFlashSectorSize;
  for (uint32_t addr = start_addr; addr < end_addr; addr += SpiFlashPageSize) {
    // Fill `read_values` with dummy data so we know a change actually occurs.
    constexpr uint8_t DummyReadData = 0x98;
    read_values.fill(DummyReadData);

    // Read the values from the specified sector in flash.
    // Sectors should be page-aligned, so we always just read full pages.
    spi_flash.read(addr, read_values.data(), SpiFlashPageSize);

    // Check that the values that were read were properly erased.
    for (auto data : read_values) {
      if (data != 0xFF) failures++;
    }
  }

  return failures;
}

/**
 * Write random values to a page of flash. The size of this page is
 * retrieved from the `SpiFlashPageSize` definition. Then reads back
 * all the written values, and checks that the values that are read matches
 * the values that were written. Assumes that the SPI has already been
 * appropriately configured. The address is assumed to be page-aligned.
 * Returns the number of failures during the test.
 */
static int spi_flash_random_data_test_page(Capability<volatile SonataSpi> spi, ds::xoroshiro::P32R8 &prng,
                                           SpiFlash spi_flash, uint32_t start_addr) {
  int failures = 0;

  std::array<uint8_t, SpiFlashPageSize> write_values, read_values;
  constexpr uint8_t DummyReadData = 0xA3;
  read_values.fill(DummyReadData);

  // Erase the sector containing the page, and then check the page was erased.
  uint32_t sector_addr = start_addr - (start_addr % SpiFlashSectorSize);
  spi_flash.erase_sector(sector_addr);
  spi_flash.read(start_addr, read_values.data(), SpiFlashPageSize);
  for (auto &read_value : read_values) {
    if (read_value != 0xFF) failures++;
  }

  // Generate random values to fill the pages, write them, and read them back
  for (auto &write_value : write_values) {
    write_value = prng();
  }
  spi_flash.write_page(start_addr, write_values.data());
  spi_flash.read(start_addr, read_values.data(), SpiFlashPageSize);

  // Check that the values that were read match the values that were written
  for (size_t index = 0; index < SpiFlashPageSize; index++) {
    if (read_values[index] != write_values[index]) {
      failures++;
    }
  }

  return failures;
}

/**
 * Test the SPI by reading the Jedec ID of the SPI Flash and comparing it to a
 * known value.
 * Returns the number of failures during the test.
 */
int spi_read_flash_jedec_id_test(Capability<volatile SonataSpi> spi, SpiFlash spi_flash) {
  int failures = 0;

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Read the JEDEC ID from Flash
  uint8_t jedec_id[3] = {0};
  spi_flash.read_jedec_id(jedec_id);

  // Check that the retrieved ID matches our expected value
  for (size_t index = 0; index < 3; index++) {
    if (jedec_id[index] != ExpectedSpiFlashJedecId[index]) {
      failures++;
    }
  }

  return failures;
}

/**
 * Erases a variety of random sectors, and then reads them back to check the
 * values that are read match the written values. This is done one sector at
 * a time. The number of tested sectors is configured by the
 * `SPI_TEST_FLASH_SECTORS_ERASED` definition.
 * Returns the number of failures during the test.
 */
int spi_flash_erase_test(Capability<volatile SonataSpi> spi, ds::xoroshiro::P32R8 &prng, SpiFlash spi_flash) {
  int failures = 0;

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Generate random sectors and test them.
  for (size_t i = 0; i < SPI_TEST_FLASH_SECTORS_ERASED; i++) {
    const uint32_t addr = random_flash_sector_addr(prng);
    failures += spi_flash_erase_test_sector(spi, prng, spi_flash, addr);
  }

  return failures;
}

/**
 * Writes random values to a variety of random pages, and then reads them
 * back to check the values that are read match the written values. This
 * is done one page at a time. The number of tested pages is configured
 * by the `SPI_TEST_FLASH_PAGES_WRITTEN` definition.
 * Returns the number of failures during the test.
 */
int spi_flash_random_data_test(Capability<volatile SonataSpi> spi, ds::xoroshiro::P32R8 &prng, SpiFlash spi_flash) {
  int failures = 0;

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Generate random pages and test them.
  for (size_t i = 0; i < SPI_TEST_FLASH_PAGES_WRITTEN; i++) {
    const uint32_t addr = random_flash_page_addr(prng);
    failures += spi_flash_random_data_test_page(spi, prng, spi_flash, addr);
  }

  return failures;
}

/**
 * This test performs a simple page write with random data as in the random data
 * test, but does it using a SPI running at 1/8th the speed, to catch possible
 * issues with the SPI when running at a slower speed.
 * Returns the number of failures during the test.
 */
int spi_flash_slow_clock_test(Capability<volatile SonataSpi> spi, ds::xoroshiro::P32R8 &prng, SpiFlash spi_flash) {
  int failures = 0;

  // Configure the SPI to be MSB first, and run such that 1 SPI clock period is
  // 16 system cycles.
  spi->wait_idle();
  spi->init(false, false, true, 8);

  // Generate a random page and test it on the slow clock speed.
  const uint32_t addr = random_flash_page_addr(prng);
  failures += spi_flash_random_data_test_page(spi, prng, spi_flash, addr);

  return failures;
}

/**
 * Run the whole suite of SPI tests.
 */
void spi_tests(CapRoot root, UartPtr console) {
  // Create bounded capabilities for SPI.
  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS;
  spi.bounds()                       = SPI_BOUNDS;

  SpiFlash spi_flash(spi);

  // Initialise 8-bit PRNG for use in random test data
  ds::xoroshiro::P32R8 prng;
  prng.set_state(0xDEAD, 0xBEEF);

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < SPI_TEST_ITERATIONS; i++) {
    write_str(console, "\r\nrunning spi_test: ");
    write_hex8b(console, i);
    write_str(console, "\\");
    write_hex8b(console, SPI_TEST_ITERATIONS - 1);
    write_str(console, "\r\n");

    bool test_failed = false;
    int failures     = 0;

    write_str(console, "  Running Flash Jedec ID Read test... ");
    failures = spi_read_flash_jedec_id_test(spi, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(console, failures);

    write_str(console, "  Running Flash Sector Erase test... ");
    failures = spi_flash_erase_test(spi, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(console, failures);

    write_str(console, "  Running Flash Random Data test... ");
    failures = spi_flash_random_data_test(spi, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(console, failures);

    write_str(console, "  Running Slow Clock test... ");
    failures = spi_flash_slow_clock_test(spi, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(console, failures);

    check_result(console, !test_failed);
  }
}
