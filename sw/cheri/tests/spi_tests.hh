// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "../../common/defs.h"
#include "../common/console.hh"
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

/**
 * Configures which of the SPI controllers shall use an external loopback
 * via a jumper cable, and not just the internal loopback within the SPI
 * block itself. (-1 = no jumper cable present)
 */
#ifndef SPI_TEST_EXT_LOOPBACK_CONN
// Try an external loopback test with another SPI controller;
// Note: we can install a loopback wire between pins MB3 and MB4 of the
// mikroBUS Click header in CI and test this using SPI controller 4.
// #define SPI_TEST_EXT_LOOPBACK_CONN (4)
#define SPI_TEST_EXT_LOOPBACK_CONN (-1)
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
 * SPI controller interrupt test.
 */
int spi_irq_test(SpiPtr spi, ds::xoroshiro::P32R8 &prng, Log &log) {
  // Ensure that the SPI SCLK speed is substantially slower than the system clock
  // so that the CPU has time to check the interrupt state immediately after
  // hitting the `start` button.
  const uint32_t kSpeed = 8u;
  const bool logging    = false;
  int failures          = 0;

  // Choose some watermark levels.
  const uint8_t encRxWatermark = prng() % 5;
  const uint8_t encTxWatermark = prng() % 4;
  const uint8_t rxWatermark    = 1u << encRxWatermark;
  const uint8_t txWatermark    = 1u << encTxWatermark;

  spi->wait_idle();
  spi->init(false, false, true, kSpeed);
  spi->control = SonataSpi::ControlInternalLoopback | SonataSpi::ControlTransmitEnable |
                 SonataSpi::ControlReceiveEnable | (encRxWatermark << 8) | (encTxWatermark << 4);

  // Ascertain the FIFO sizes.
  const uint8_t rxFifoSize = (spi->info & SonataSpi::InfoRxFifoDepth) >> 8;
  const uint8_t txFifoSize = (spi->info & SonataSpi::InfoTxFifoDepth);

  if (logging) {
    log.println("rxWatermark {} txWatermark {} rxFifoSize {} txFifoSize {}", rxWatermark, txWatermark, rxFifoSize,
                txFifoSize);
  }

  // The test consists of a number of phases.
  enum { PhaseFillTx, PhaseTransfer, PhaseDrainRx, PhaseComplete } testPhase = PhaseFillTx;
  // Each test phase needs to count iterations.
  unsigned iter = 0u;

  // The data values do not matter for this test.
  const uint8_t testDatum = 0xaau;
  // Expected state of the FIFOs.
  uint32_t expRxLevel = 0u;
  uint32_t expTxLevel = 0u;
  do {
    // Derived, expected status.
    bool expRxWatermark = (expRxLevel >= rxWatermark);
    bool expTxWatermark = (expTxLevel <= txWatermark);
    bool expRxFull      = (expRxLevel >= rxFifoSize);
    bool expTxFull      = (expTxLevel >= txFifoSize);
    bool expRxEmpty     = !expRxLevel;
    bool expTxEmpty     = !expTxLevel;

    // Read the current state information from the SPI controller.
    uint32_t interruptState = spi->interruptState;
    bool actRxWatermark     = (SonataSpi::InterruptReceiveWatermark & interruptState) != 0u;
    bool actTxWatermark     = (SonataSpi::InterruptTransmitWatermark & interruptState) != 0u;
    bool actTxEmpty         = (SonataSpi::InterruptTransmitEmpty & interruptState) != 0u;
    bool actRxFull          = (SonataSpi::InterruptReceiveFull & interruptState) != 0u;
    uint32_t status         = spi->status;
    uint8_t actRxLevel      = (SonataSpi::StatusRxFifoLevel & status) >> 8;
    uint8_t actTxLevel      = (SonataSpi::StatusTxFifoLevel & status);
    bool actRxEmpty         = (SonataSpi::StatusRxFifoEmpty & status) != 0u;
    bool actTxFull          = (SonataSpi::StatusTxFifoFull & status) != 0u;

    if (logging) {
      log.println("status {} interruptState {}", status, interruptState);
    }

    // Check that the FIFO levels are as expected.
    failures += (actRxLevel != expRxLevel);
    failures += (actTxLevel != expTxLevel);
    // Check that interrupt state indicators are as expected.
    failures += (actRxEmpty != expRxEmpty);
    failures += (actTxEmpty != expTxEmpty);
    failures += (actRxFull != expRxFull);
    failures += (actTxFull != expTxFull);
    // Check that the watermark interrupts are as expected.
    failures += (actRxWatermark != expRxWatermark);
    failures += (actTxWatermark != expTxWatermark);

    switch (testPhase) {
      // Filling the Tx FIFO.
      case PhaseFillTx:
        if (iter++ < txFifoSize) {
          spi->transmitFifo = testDatum;
          expTxLevel++;
        } else {
          testPhase = PhaseTransfer;
          iter      = 0u;
        }
        break;

      // Transferring data from Tx FIFO to Rx FIFO using the controller.
      case PhaseTransfer:
        if (iter++ < txFifoSize) {
          // Ensure that the `complete` interrupt is clear.
          spi->interruptState = static_cast<uint32_t>(SonataSpi::InterruptComplete);

          // Shuffle a single byte from the Tx FIFO to the Rx FIFO.
          spi->start = 1u;
          // Check that the interrupt is still clear; the SPI controller is limited
          // by its programmed SCLK speed, but we need to perform this test promptly.
          failures += (SonataSpi::InterruptComplete & spi->interruptState) != 0u;
          spi->wait_idle();

          // Check that the `complete` interrupt has become set, before clearing it.
          failures += !(SonataSpi::InterruptComplete & spi->interruptState);
          spi->interruptState = static_cast<uint32_t>(SonataSpi::InterruptComplete);

          // A single byte should have been moved.
          expTxLevel--;
          expRxLevel++;
        } else {
          testPhase = PhaseDrainRx;
          iter      = 0u;
        }
        break;

      // Draining the data from the Rx FIFO.
      case PhaseDrainRx:
        if (iter++ < rxFifoSize) {
          // Remove a byte from the Rx FIFO; may as well check its value.
          failures += (testDatum != spi->receiveFifo);
          expRxLevel--;
        } else {
          testPhase = PhaseComplete;
        }
        break;

      // Invalid/undefined state.
      default: {
        testPhase = PhaseComplete;
        failures++;
      } break;
    }
  } while (testPhase != PhaseComplete);

  return failures;
}

/**
 * SPI loopback test transmits a sequence of pseudo-random bytes and checks that
 * the same data is received correctly with transmit and receive enabled correctly.
 *
 * This test may be performed using internal loopback on any of the SPI controllers
 * and, to gain additional confidence of the pinmux/connectivity, optionally using
 * an external loopback via a jumper cable, eg. on the mikroBUS Click header between
 * pins MB3 and MB4, or on the Raspberry Pi header between pins 19 and 20.
 */
int spi_loopback_test(SpiPtr spi, bool external, bool cpol, bool cpha, bool msb_first, ds::xoroshiro::P32R8 &prng,
                      Log &log) {
  constexpr uint32_t kSpiSpeed = 0u;  // Let's go as fast as possible.
  // Take a copy of the PRNG so that we can predict the read-side data.
  ds::xoroshiro::P32R8 read_prng = prng;
  size_t bytes_read              = 0u;
  size_t bytes_sent              = 0u;
  // Number of bytes to be transferred; remember that we can only supply 0x7ffu bytes
  // in a single operation.
  size_t len   = 0x700u;
  int failures = 0;
  bool logging = false;

  spi->wait_idle();
  spi->init(cpol, cpha, msb_first, kSpiSpeed);

  spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
  if (!external) {
    // Enable the internal loopback function.
    spi->control = spi->control | SonataSpi::ControlInternalLoopback;
  }
  spi->start = len;

  // Repeat until all bytes have been received, which should imply that all bytes have
  // been sent, but we'll check that after the test...
  while (bytes_read < len) {
    // Can we send another byte yet?
    if (bytes_sent < len && !(SonataSpi::StatusTxFifoFull & spi->status)) {
      uint8_t b         = static_cast<uint8_t>(prng());
      spi->transmitFifo = b;
      bytes_sent++;
      if (logging) {
        log.println("sent {:#02x}", b);
      }
    }
    // Is there any data to read and check?
    if (!(SonataSpi::StatusRxFifoEmpty & spi->status)) {
      // Check the received data against the transmitted data.
      uint8_t act_byte = static_cast<uint8_t>(spi->receiveFifo);
      uint8_t exp_byte = read_prng();
      failures += (act_byte != exp_byte);
      bytes_read++;
      if (logging) {
        log.println("expected {:#02x} read {:#02x}", exp_byte, act_byte);
      }
    }
  }

  // Check that we also sent the expected number of bytes, and that nothing remains.
  failures += (bytes_sent != bytes_read);
  failures += (spi->status & SonataSpi::StatusTxFifoLevel) != 0;
  failures += (spi->status & SonataSpi::StatusRxFifoLevel) != 0;
  failures += !(spi->status & SonataSpi::StatusIdle);

  if (logging) {
    log.println("failures: {}", failures);
  }

  return failures;
}

/**
 * Run the whole suite of SPI tests.
 */
void spi_tests(CapRoot root, Log &log) {
  // Create bounded capabilities for SPI.
  Capability<volatile SonataSpi> spi0 = root.cast<volatile SonataSpi>();
  spi0.address()                      = SPI_ADDRESS;
  spi0.bounds()                       = SPI_BOUNDS;

  // Access to each of the SPI controllers.
  SpiPtr spis[SPI_NUM];
  for (int s = 0; s < SPI_NUM; s++) {
    spis[s] = spi_ptr(root, s);
    // Clear any legacy of previous tests; empty FIFOs and reset the
    // controller core.
    spis[s]->init(false, false, true, 0);
  }

  SpiFlash spi_flash(spi0);

  // Initialise 8-bit PRNG for use in random test data
  ds::xoroshiro::P32R8 prng;
  prng.set_state(0xDEAD, 0xBEEF);

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < SPI_TEST_ITERATIONS; i++) {
    log.println("\r\nrunning spi_test: {} \\ {}", i, SPI_TEST_ITERATIONS - 1);

    bool test_failed = false;
    int failures     = 0;

    log.print("  Running Flash Jedec ID Read test... ");
    failures = spi_read_flash_jedec_id_test(spi0, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running Flash Sector Erase test... ");
    failures = spi_flash_erase_test(spi0, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running Flash Random Data test... ");
    failures = spi_flash_random_data_test(spi0, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    log.print("  Running Slow Clock test... ");
    failures = spi_flash_slow_clock_test(spi0, prng, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    // Loopback testing and IRQ testing.
    for (int s = 0; s < SPI_NUM; s++) {
      // Default configuration.
      const bool msb_first = true;
      const bool cpha      = false;
      const bool cpol      = false;

      log.print("  Running SPI {} internal loopback test... ", s);

      // Internal loopback test.
      failures = spi_loopback_test(spis[s], false, cpol, cpha, msb_first, prng, log);
      test_failed |= (failures > 0);
      write_test_result(log, failures);

      // Do we have an external loopback on this controller?
      if (SPI_TEST_EXT_LOOPBACK_CONN == s) {
        log.print("  Running SPI {} external loopback test... ", s);

        failures = spi_loopback_test(spis[s], true, cpol, cpha, msb_first, prng, log);
        test_failed |= (failures > 0);
        write_test_result(log, failures);
      }

      // Interrupt test; this also uses the loopback functionality so it should follow that.
      log.print("  Running SPI {} IRQ test... ", s);
      failures = spi_irq_test(spis[s], prng, log);
      test_failed |= (failures > 0);
      write_test_result(log, failures);
    }

    // Check all polarities and phases; we'll use the highest-numbered
    // SPI controller this time just to minimise the chance of
    // unintended iteraction with physical devices.
    // (Chip Select is not being asserted, but still...)
    for (unsigned cfg = 0u; cfg <= 7u; ++cfg) {
      const unsigned spi_num = SPI_NUM - 1u;
      const bool msb_first   = (cfg & 4u) != 0u;
      const bool cpha        = (cfg & 2u) != 0u;
      const bool cpol        = (cfg & 1u) != 0u;

      log.print("  Running SPI {} with config {}... ", spi_num, cfg);
      failures = spi_loopback_test(spis[spi_num], false, cpol, cpha, msb_first, prng, log);
      test_failed |= (failures > 0);
      write_test_result(log, failures);
    }

    check_result(log, !test_failed);
  }
}
