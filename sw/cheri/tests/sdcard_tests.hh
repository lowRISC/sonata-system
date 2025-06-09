/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <assert.h>
#include <ctype.h>
#include <stdint.h>

#include <platform-spi.hh>

#include "../../common/defs.h"

#include "../common/console.hh"
#include "../common/filesys-utils.hh"
#include "../common/platform-pinmux.hh"
#include "../common/sdcard-utils.hh"
#include "../common/sonata-devices.hh"

#include "../tests/test_runner.hh"

// Lorem Ipsum sample text.
#include "lorem_text.hh"

#define MAX_BLOCKS 0x10
#define BLOCK_LEN 0x200

// Set this for manual operation rather than automated regression testing.
static constexpr bool manual = false;

// Set this to true to enable diagnostic logging.
static constexpr bool logging = false;

// Set this to true to emit the `lorem ipsum` sample text for capture and subsequent
// writing to a FAT32-formatted microSD card as `LOREM.IPS` within the root directory.
static constexpr bool emitText = false;

// Scratch workspace for reading file blocks or Long FileName.
static uint8_t fileBuffer[BLOCK_LEN];

// Compare a sequence of bytes against a reference, returning the number of mismatches.
static int compare_bytes(const char *ref, unsigned &offset, const uint8_t *data, size_t len, Log &log) {
  unsigned mismatches = 0u;
  while (len-- > 0u) {
    // Compare retrieved data byte against reference text.
    uint8_t dch = *data++;
    char ch     = ref[offset++];
    // It's quite likely that the data stored on the card is LF-terminated rather than
    // the CR,LF termination that we expect, so we permit that and continue checking.
    if ((char)dch == '\n' && ch == '\r') {
      ch = ref[offset++];
    }
    mismatches += (char)dch != ch;
  }
  return mismatches;
}

// Read and report the properties of the SD card itself (CSD and CID).
static int read_card_properties(bool &validCID, SdCard &sd, Log &log, bool logging = true) {
  int failures = 0u;
  uint8_t buf[16];
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("  Reading Card Specific Data (CSD) ");
  if (sd.read_csd(buf, sizeof(buf))) {
    if (logging) {
      dump_bytes(log, buf, sizeof(buf));
    }
    // The final byte contains a CRC7 field within its MSBs.
    uint8_t crc = 1u | (SdCard::calc_crc7(buf, sizeof(buf) - 1u) << 1);
    failures += (crc != buf[sizeof(buf) - 1u]);
  } else {
    failures++;
  }
  write_test_result(log, failures);

  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("  Reading Card Identification (CID) ");
  validCID = false;
  if (sd.read_cid(buf, sizeof(buf))) {
    if (logging) {
      dump_bytes(log, buf, sizeof(buf));
    }
    // The final byte contains a CRC7 field within its MSBs.
    uint8_t crc = 1u | (SdCard::calc_crc7(buf, sizeof(buf) - 1u) << 1);
    failures += (crc != buf[sizeof(buf) - 1u]);
    // Check that the manufacturer ID is non-zero and the OEM/Application ID contains two
    // valid ASCII characters.
    if (buf[0] && buf[1] >= 0x20 && buf[1] < 0x7f && buf[2] >= 0x20 && buf[2] < 0x7f) {
      validCID = true;
    }
  } else {
    failures++;
  }
  write_test_result(log, failures);

  return failures;
}

/**
 * Run the set of SD card tests; test card presence, read access to the card itself
 * and then the data stored within the flash. The test expects a FAT32-formatted
 * SD card with a sample file called `LOREM.IPS` in the root directory.
 */
void sdcard_tests(CapRoot &root, Log &log) {
  // Have we been asked to emit the sample text?
  if (emitText) {
    log.println(
        "Capture everything between the dotted lines, being careful not "
        "to introduce any additional line breaks.");
    log.println("--------");
    log.print(lorem_text);
    log.println("--------");
    log.println(
        "Each of these single-line paragraphs shall be CR,LF terminated "
        "and followed by a blank line.");
    log.println("This includes the final one, and thus the file itself ends with a blank line.");
    log.println("The file should be 4,210 bytes in length.");
  }

  // The SPI controller talks to the microSD card in SPI mode.
  auto spi = spi_ptr(root, 2);

  // We need to use the pinmux to select the microSD card for SPI controller 2 reads (CIPO),
  // as well as preventing outbound traffic to the microSD card also reaching the application
  // flash (for safety; it _should_ ignore traffic not accompanied by Chip Select assertion).
  auto pin_output                 = pin_sinks_ptr(root);
  SonataPinmux::Sink appspi_cs    = pin_output->get(SonataPinmux::PinSink::appspi_cs);
  SonataPinmux::Sink appspi_clk   = pin_output->get(SonataPinmux::PinSink::appspi_clk);
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

  // Initialise SD card access, using CRCs on all traffic.
  SdCard sd(spi, gpio, csBit, detBit, true);

  int failures = 0u;
  if (!sd.present()) {
    if (manual) {
      // Wait until a card is detected.
      log.println("Please insert a microSD card into the slot...");
      while (!sd.present());
    } else {
      log.println("No microSD card detected");
      failures++;
    }
  }
  if (sd.present()) {
    bool validCID;
    sd.init();

    log.println("Reading card properties.... ");
    failures += read_card_properties(validCID, sd, log);

    // The CI system presently does not have a valid microSD card image and properties in
    // simulation. We have already tested SPI traffic so if we haven't read a valid CID,
    // skip the block level/filing system testing.
    if (validCID) {
      log.println("Reading card contents.... ");
      fileSysUtils fs;

      failures += !fs.init(&sd);
      write_test_result(log, failures);

      if (!failures) {
        // List the files and subdirectories in the root directory.
        log.println("Reading root directory.... ");
        fileSysUtils::dirHandle dh = fs.rootdir_open();
        if (dh == fileSysUtils::kInvalidDirHandle) {
          failures++;
        } else {
          uint16_t *ucs        = reinterpret_cast<uint16_t *>(fileBuffer);
          const size_t ucs_max = sizeof(fileBuffer) / 2;
          fileSysUtils::dirEntry entry;
          while (fs.dir_next(dh, entry, fileSysUtils::DirFlags_Default, ucs, ucs_max)) {
            log.print("'");
            write_str_ucs2(log, ucs, ucs_max);
            log.println("' : length {:#x} cluster {:#x}", entry.dataLength, entry.firstCluster);
          }
          fs.dir_close(dh);
        }
        write_test_result(log, failures);

        // Locate and check the LOREM.IPS test file in the root directory.
        fileSysUtils::fileHandle fh = fs.file_open("lorem.ips");
        if (fh == fileSysUtils::kInvalidFileHandle) {
          log.println("Unable to locate file");
          failures++;
        } else {
          // Determine the length of the file.
          ssize_t fileLen = fs.file_length(fh);
          if (fileLen < 0) {
            log.println("Failed to read file length");
            failures++;
          } else {
            log.println("File is {} byte(s)", fileLen);
          }
          uint32_t sampleOffset = 0u;
          while (fileLen > 0 && sampleOffset < sizeof(lorem_text)) {
            // Work out how many bytes we can compare.
            uint32_t chunkLen = (fileLen >= sizeof(fileBuffer)) ? sizeof(fileBuffer) : fileLen;
            if (chunkLen > sizeof(lorem_text) - sampleOffset) {
              chunkLen = sizeof(lorem_text) - sampleOffset;
            }
            // Read data from the SD card into our buffer.
            size_t read = fs.file_read(fh, fileBuffer, chunkLen);
            if (read != chunkLen) {
              // We did not read the expected number of bytes.
              log.println("File read did not return the requested number of bytes");
              failures++;
            }
            if (logging) {
              dump_bytes(log, fileBuffer, chunkLen);
            }
            // Compare this data against the sample text.
            failures += compare_bytes(lorem_text, sampleOffset, fileBuffer, chunkLen, log);
            fileLen -= chunkLen;
          }
          log.println("Done text comparison");
          // If we have not compared the entire file, count that as a failure.
          failures += (fileLen > 0);
          fs.file_close(fh);
        }
        write_test_result(log, failures);
      } else {
        log.println("No valid Master Boot Record found (signature not detected)");
        failures++;
      }
    }
  }
  write_test_result(log, failures);
  check_result(log, !failures);

  // Be a good citizen and put the pinmux back in its default state.
  microsd_dat3.disable();
  microsd_clk.disable();
  microsd_cmd.disable();
  // Suppress traffic to the application flash.
  appspi_cs.default_selection();
  appspi_clk.default_selection();
  appspi_d0.default_selection();
  // Direct SPI controller 2 to drive the microSD pins.
  // Select microSD CIPO as SPI controller input.
  constexpr uint8_t PmuxSpi0CipoToAppSpiD1 = 1;
  spi_0_cipo.select(PmuxSpi0CipoToAppSpiD1);
}
