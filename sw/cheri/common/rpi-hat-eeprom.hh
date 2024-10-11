// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdint.h>

// Functions for working with ID EEPROMs on R-PI HATs. Specification available
// in Appendix A of the R-PI HAT+ specification:
// https://datasheets.raspberrypi.com/hat/hat-plus-specification.pdf

namespace RpiHatEeprom {

typedef struct {
  uint16_t type;
  uint16_t atom_count;
  uint32_t length;
  uint16_t crc;
} atom_info_t;

constexpr uint16_t AtomHeaderLength = 8;

typedef struct {
  uint8_t uuid[16];
  uint16_t pid;
  uint16_t pver;
  uint8_t vendor_str_len;
  uint8_t product_str_len;
} vendor_atom_t;

constexpr uint16_t VendorHeaderLength = 22;

typedef struct {
  char signature[4];
  uint8_t version;
  uint16_t num_atoms;
  uint32_t length;
} eeprom_header_t;

enum AtomType : uint16_t {
  Invalid      = 0,
  VendorInfo   = 1,
  GPIOBank0Map = 2,  // Deprecated in HAT+ specification
  DeviceTree   = 3
};

constexpr uint16_t EepromHeaderLength = 12;

constexpr uint16_t Crc16Poly = 0xA001;

static uint16_t calc_crc16(const uint8_t buf[], uint32_t start_index, uint32_t len) {
  uint16_t crc = 0;

  for (uint32_t i = start_index; i < start_index + len; ++i) {
    crc ^= static_cast<uint16_t>(buf[i]);

    for (int b = 0; b < 8; ++b) {
      uint16_t bottom_bit = crc & 0x1;

      crc >>= 1;

      if (bottom_bit != 0) {
        crc = crc ^ Crc16Poly;
      }
    }
  }

  return crc;
}

static uint16_t read_buf_16b(const uint8_t buf[], uint32_t start_index) {
  return static_cast<uint16_t>(buf[start_index]) | (static_cast<uint16_t>(buf[start_index + 1]) << 8);
}

static uint32_t read_buf_32b(const uint8_t buf[], uint32_t start_index) {
  return static_cast<uint32_t>(buf[start_index]) | (static_cast<uint32_t>(buf[start_index + 1]) << 8) |
         (static_cast<uint32_t>(buf[start_index + 2]) << 16) | (static_cast<uint32_t>(buf[start_index + 3]) << 24);
}

// Reads overall EEPROM header from provided buffer and validates it. The header
// must begin at index 0.
// Return false if header data is invalid or if buffer isn't sufficiently large
// to hold the eeprom ID contents (as determined by the overal length in the
// header).
static bool parse_header(const uint8_t buf[], uint32_t buf_size, eeprom_header_t& header_out) {
  if (buf_size < 12) {
    return false;
  }

  header_out.signature[0] = buf[0];
  header_out.signature[1] = buf[1];
  header_out.signature[2] = buf[2];
  header_out.signature[3] = buf[3];

  header_out.version   = buf[4];
  header_out.num_atoms = read_buf_16b(buf, 6);
  header_out.length    = read_buf_32b(buf, 8);

  if (!(header_out.signature[0] == 'R' && header_out.signature[1] == '-' && header_out.signature[2] == 'P' &&
        header_out.signature[3] == 'i')) {
    return false;
  }

  if (buf_size < header_out.length) {
    return false;
  }

  return true;
}

// Reads header and footer information for an atom from the buffer beginning at
// 'start_index'.
// Returns false if buffer is too small to contain the entire atom (as
// determined by the size provided in the atom header) or if the CRC is not
// correct.
static bool parse_atom_info(const uint8_t buf[], uint32_t start_index, uint32_t buf_size, atom_info_t& info_out) {
  // Smallest atom (0 byte data payload) has header + 2 byte CRC
  if ((start_index + AtomHeaderLength + 2) > buf_size) {
    return false;
  }

  info_out.type       = read_buf_16b(buf, start_index);
  info_out.atom_count = read_buf_16b(buf, start_index + 2);
  info_out.length     = read_buf_32b(buf, start_index + 4);

  // Check buffer long enough to contain entire atom
  if ((start_index + info_out.length) > buf_size) {
    return false;
  }

  // Length is size of data payload + CRC so calculate end of atom with known
  // lengths offset by -2 to get CRC index.
  info_out.crc = read_buf_16b(buf, start_index + AtomHeaderLength + info_out.length - 2);

  uint16_t calc_crc = calc_crc16(buf, start_index, AtomHeaderLength + info_out.length - 2);

  if (calc_crc != info_out.crc) {
    return false;
  }

  return true;
}

// Reads header for a vendor info atom.
// Returns false if vendor and product string do not fit in the buffer (using
// the string lengths provided in the header).
static bool parse_vendor_info(const uint8_t buf[], uint32_t start_index, uint32_t buf_size,
                              vendor_atom_t& vendor_atom_out) {
  if ((start_index + 22) > buf_size) {
    return false;
  }

  for (int i = 0; i < 16; ++i) {
    vendor_atom_out.uuid[i] = buf[start_index + i];
  }

  vendor_atom_out.pid             = read_buf_16b(buf, start_index + 16);
  vendor_atom_out.pver            = read_buf_16b(buf, start_index + 18);
  vendor_atom_out.vendor_str_len  = buf[start_index + 20];
  vendor_atom_out.product_str_len = buf[start_index + 21];

  if ((start_index + VendorHeaderLength + vendor_atom_out.vendor_str_len + vendor_atom_out.product_str_len) >
      buf_size) {
    return false;
  }

  return true;
}

};  // namespace RpiHatEeprom
