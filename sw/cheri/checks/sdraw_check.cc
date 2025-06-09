/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * SD card raw write/read block test; this is a MANUAL check that should be performed only
 * on a card THAT DOES NOT CONTAIN ANY DATA OF VALUE.
 *
 * It does not require formatting or any particular file system to be present.
 *
 * If there is a card inserted when the test starts up, you will be asked to remove the
 * card and then insert it after the warning message is displayed; this is to decrease the
 * likelihood of inadvertent data loss.
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../common/sonata-devices.hh"
// clang-format off
#include <ds/xoroshiro.h>
#include <cheri.hh>
// clang-format on
#include "../common/console.hh"
#include "../common/sdcard-utils.hh"
#include "../common/uart-utils.hh"

// Maximum number of blocks per transfer; reasonable model of FS activity.
static constexpr unsigned kMaxBlocks = 0x10u;
// Must be a power-of-two.
static_assert(!(kMaxBlocks & (kMaxBlocks - 1u)));

static constexpr unsigned kBlockLen = 0x200u;

// Diagnostic logging?
static const bool kLogging = false;

static uint8_t write_data[kMaxBlocks * kBlockLen];
static uint8_t read_data[kMaxBlocks * kBlockLen];
static uint8_t orig_data[kMaxBlocks * kBlockLen];

// Compare a sequence of bytes against a reference, returning the number of mismatches.
static int compare_bytes(const uint8_t *ref, const uint8_t *data, size_t len, Log &log) {
  unsigned mismatches = 0u;
  while (len-- > 0u) {
    mismatches += *ref++ != *data++;
  }
  return mismatches;
}

extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  CapRoot root{rwRoot};

  auto uart0 = uart_ptr(root);
  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);

  // The SPI controller talks to the microSD card in SPI mode.
  auto spi = spi_ptr(root, 2);

  // We need to use the pinmux to select the microSD card for SPI controller 2 reads (CIPO),
  // as well as preventing outbound traffic to the microSD card also reaching the application
  // flash (for safety; it _should_ ignore traffic not accompanied by Chip Select assertion).
  auto pin_output                 = pin_sinks_ptr(root);
  SonataPinmux::Sink appspi_cs    = pin_output->get(SonataPinmux::PinSink::appspi_cs);
  SonataPinmux::Sink appspi_clk   = pin_output->get(SonataPinmux::PinSink::appspi_cs);
  SonataPinmux::Sink appspi_d0    = pin_output->get(SonataPinmux::PinSink::appspi_d0);
  SonataPinmux::Sink microsd_dat3 = pin_output->get(SonataPinmux::PinSink::microsd_dat3);
  SonataPinmux::Sink microsd_clk  = pin_output->get(SonataPinmux::PinSink::microsd_clk);
  SonataPinmux::Sink microsd_cmd  = pin_output->get(SonataPinmux::PinSink::microsd_cmd);

  auto block_input              = block_sinks_ptr(root);
  SonataPinmux::Sink spi_0_cipo = block_input->get(SonataPinmux::BlockSink::spi_0_cipo);

  // Suppress traffic to the application flash.
  appspi_cs.disable();
  appspi_clk.disable();
  appspi_d0.disable();
  // Direct SPI controller 2 to drive the microSD pins.
  microsd_dat3.default_selection();
  microsd_clk.default_selection();
  microsd_cmd.default_selection();
  // Select microSD CIPO as SPI controller input.
  constexpr uint8_t PmuxSpi0CipoToSdDat0 = 2;
  spi_0_cipo.select(PmuxSpi0CipoToSdDat0);

  // We need to use the GPIO to detect card presence.
  auto gpio = gpio_ptr(root);

  // microSD card is on Chip Select 1 (0 goes to the application flash).
  constexpr unsigned csBit = 1u;
  // microSD card detection bit is on input 16.
  constexpr unsigned detBit = 16u;

  log.println("SD card raw write/read test.");

  SdCard sd(spi, gpio, csBit, detBit, true);

  // Wait until there is no card in the slot before issuing a warning message.
  if (sd.present()) {
    log.println("Please remove the microSD card from the slot.");
    while (sd.present()) {
      // Prevent erroneous optimisation to infinite loop (.l: j l)
      asm("");
    }
  }
  // Wait until a card is detected.
  if (!sd.present()) {
    log.println("Please insert a microSD card that does not contain any valued data.");
    log.println("*** DATA BLOCKS WILL BE OVERWRITTEN ***");
    while (!sd.present()) {
      // Prevent erroneous optimisation to infinite loop (.l: j l)
      asm("");
    }
  }

  log.println("Starting write/read test.... ");

  sd.init();

  // Initialise random number generation.
  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);

  const unsigned numOps = 0x10u;
  int failures          = 0;
  for (unsigned b = 0u; !failures && b < numOps; b++) {
    // Choose a random initial block.
    uint32_t blk = (uint16_t)prng();
    // Choose a random transfer length.
    unsigned num = 1 + ((kMaxBlocks - 1u) & prng());
    log.println("Testing {} block(s) from block {} onwards", num, blk);
    // Collect the data from those blocks.
    log.println("Reading original data...");
    failures += !sd.read_blocks(blk, orig_data, num);
    if (!failures) {
      if (kLogging) {
        log.println("Original data:");
        dump_bytes(log, orig_data, num * kBlockLen);
      }
      // Randomise the data that we're going to write.
      for (unsigned idx = 0u; idx < num * kBlockLen; idx++) {
        write_data[idx] = (uint8_t)prng();
      }
      if (kLogging) {
        log.println("Write data:");
        dump_bytes(log, write_data, num * kBlockLen);
      }
      log.println("Writing...");
      // Write out the data to the chosen block.
      failures += !sd.write_blocks(blk, write_data, num);
      if (!failures) {
        log.println("Reading back...");
        // Read it back.
        failures += !sd.read_blocks(blk, read_data, num);
        if (kLogging) {
          log.println("Read data:");
          dump_bytes(log, read_data, num * kBlockLen);
        }
        // Check each of the read bytes against what we tried to write.
        failures += compare_bytes(write_data, read_data, num * kBlockLen, log);
        // Try to put the original data back even if there was a failure.
        log.println("Restoring...");
        failures += !sd.write_blocks(blk, orig_data, num);
      }
    }
  }
  write_test_result(log, failures);

  while (true) {
    asm("");
  }
}
