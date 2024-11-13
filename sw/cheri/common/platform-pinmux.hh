// SPDX-FileCopyrightText: lowRISC contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <debug.hh>
#include <stdint.h>
#include <utils.hh>
#include <cheri.hh>

namespace SonataPinmux {
/// The number of pin sinks (pin outputs)
static constexpr size_t NumPinSinks = 85;

/// The number of block sinks (block inputs)
static constexpr size_t NumBlockSinks = 70;

/// Flag to set when debugging the driver for UART log messages.
static constexpr bool DebugDriver = false;

/// Helper for conditional debug logs and assertions.
using Debug = ConditionalDebug<DebugDriver, "Pinmux">;

/// The disable bit that disables a sink
constexpr uint8_t SourceDisabled = 0;

/// The bit that resets a sink to it's default value
constexpr uint8_t SourceDefault = 1;

/**
 * Each pin sink is configured by an 8-bit register. This enum maps pin sink
 * names to the offset of their configuration registers. The offsets are relative
 * to the first pin sink register.
 *
 * Documentation sources:
 * 1. https://lowrisc.github.io/sonata-system/doc/ip/pinmux/
 * 2. https://github.com/lowRISC/sonata-system/blob/v1.0/data/pins_sonata.xdc
 * 3. https://github.com/newaetech/sonata-pcb/blob/649b11c2fb758f798966605a07a8b6b68dd434e9/sonata-schematics-r09.pdf
 */
enum class PinSink : uint16_t {
  ser0_tx      = 0x000,
  ser1_tx      = 0x001,
  rs232_tx     = 0x002,
  rs485_tx     = 0x003,
  scl0         = 0x004,
  sda0         = 0x005,
  scl1         = 0x006,
  sda1         = 0x007,
  rph_g0       = 0x008,
  rph_g1       = 0x009,
  rph_g2_sda   = 0x00a,
  rph_g3_scl   = 0x00b,
  rph_g4       = 0x00c,
  rph_g5       = 0x00d,
  rph_g6       = 0x00e,
  rph_g7       = 0x00f,
  rph_g8       = 0x010,
  rph_g9       = 0x011,
  rph_g10      = 0x012,
  rph_g11      = 0x013,
  rph_g12      = 0x014,
  rph_g13      = 0x015,
  rph_txd0     = 0x016,
  rph_rxd0     = 0x017,
  rph_g16      = 0x018,
  rph_g17      = 0x019,
  rph_g18      = 0x01a,
  rph_g19      = 0x01b,
  rph_g20      = 0x01c,
  rph_g21      = 0x01d,
  rph_g22      = 0x01e,
  rph_g23      = 0x01f,
  rph_g24      = 0x020,
  rph_g25      = 0x021,
  rph_g26      = 0x022,
  rph_g27      = 0x023,
  ah_tmpio0    = 0x024,
  ah_tmpio1    = 0x025,
  ah_tmpio2    = 0x026,
  ah_tmpio3    = 0x027,
  ah_tmpio4    = 0x028,
  ah_tmpio5    = 0x029,
  ah_tmpio6    = 0x02a,
  ah_tmpio7    = 0x02b,
  ah_tmpio8    = 0x02c,
  ah_tmpio9    = 0x02d,
  ah_tmpio10   = 0x02e,
  ah_tmpio11   = 0x02f,
  ah_tmpio12   = 0x030,
  ah_tmpio13   = 0x031,
  mb1          = 0x032,
  mb2          = 0x033,
  mb4          = 0x034,
  mb5          = 0x035,
  mb6          = 0x036,
  mb7          = 0x037,
  mb10         = 0x038,
  pmod0_1      = 0x039,
  pmod0_2      = 0x03a,
  pmod0_3      = 0x03b,
  pmod0_4      = 0x03c,
  pmod0_7      = 0x03d,
  pmod0_8      = 0x03e,
  pmod0_9      = 0x03f,
  pmod0_10     = 0x040,
  pmod1_1      = 0x041,
  pmod1_2      = 0x042,
  pmod1_3      = 0x043,
  pmod1_4      = 0x044,
  pmod1_7      = 0x045,
  pmod1_8      = 0x046,
  pmod1_9      = 0x047,
  pmod1_10     = 0x048,
  pmodc_1      = 0x049,
  pmodc_2      = 0x04a,
  pmodc_3      = 0x04b,
  pmodc_4      = 0x04c,
  pmodc_5      = 0x04d,
  pmodc_6      = 0x04e,
  appspi_d0    = 0x04f,
  appspi_clk   = 0x050,
  appspi_cs    = 0x051,
  microsd_cmd  = 0x052,
  microsd_clk  = 0x053,
  microsd_dat3 = 0x054,
};

/**
 * Each block sink is configured by an 8-bit register. This enum maps block sink
 * names to the offset of their configuration registers. The offsets are relative
 * to the first block sink register.
 *
 * For GPIO block reference:
 *   gpio_0 = Raspberry Pi Header Pins
 *   gpio_1 = Arduino Shield Header Pins
 *   gpio_2 = Pmod0 Pins
 *   gpio_3 = Pmod1 Pins
 *   gpio_4 = PmodC Pins
 *
 * Documentation source:
 * https://lowrisc.github.io/sonata-system/doc/ip/pinmux/
 */
enum class BlockSink : uint16_t {
  gpio_0_ios_0  = 0x000,
  gpio_0_ios_1  = 0x001,
  gpio_0_ios_2  = 0x002,
  gpio_0_ios_3  = 0x003,
  gpio_0_ios_4  = 0x004,
  gpio_0_ios_5  = 0x005,
  gpio_0_ios_6  = 0x006,
  gpio_0_ios_7  = 0x007,
  gpio_0_ios_8  = 0x008,
  gpio_0_ios_9  = 0x009,
  gpio_0_ios_10 = 0x00a,
  gpio_0_ios_11 = 0x00b,
  gpio_0_ios_12 = 0x00c,
  gpio_0_ios_13 = 0x00d,
  gpio_0_ios_14 = 0x00e,
  gpio_0_ios_15 = 0x00f,
  gpio_0_ios_16 = 0x010,
  gpio_0_ios_17 = 0x011,
  gpio_0_ios_18 = 0x012,
  gpio_0_ios_19 = 0x013,
  gpio_0_ios_20 = 0x014,
  gpio_0_ios_21 = 0x015,
  gpio_0_ios_22 = 0x016,
  gpio_0_ios_23 = 0x017,
  gpio_0_ios_24 = 0x018,
  gpio_0_ios_25 = 0x019,
  gpio_0_ios_26 = 0x01a,
  gpio_0_ios_27 = 0x01b,
  gpio_1_ios_0  = 0x01c,
  gpio_1_ios_1  = 0x01d,
  gpio_1_ios_2  = 0x01e,
  gpio_1_ios_3  = 0x01f,
  gpio_1_ios_4  = 0x020,
  gpio_1_ios_5  = 0x021,
  gpio_1_ios_6  = 0x022,
  gpio_1_ios_7  = 0x023,
  gpio_1_ios_8  = 0x024,
  gpio_1_ios_9  = 0x025,
  gpio_1_ios_10 = 0x026,
  gpio_1_ios_11 = 0x027,
  gpio_1_ios_12 = 0x028,
  gpio_1_ios_13 = 0x029,
  gpio_2_ios_0  = 0x02a,
  gpio_2_ios_1  = 0x02b,
  gpio_2_ios_2  = 0x02c,
  gpio_2_ios_3  = 0x02d,
  gpio_2_ios_4  = 0x02e,
  gpio_2_ios_5  = 0x02f,
  gpio_2_ios_6  = 0x030,
  gpio_2_ios_7  = 0x031,
  gpio_3_ios_0  = 0x032,
  gpio_3_ios_1  = 0x033,
  gpio_3_ios_2  = 0x034,
  gpio_3_ios_3  = 0x035,
  gpio_3_ios_4  = 0x036,
  gpio_3_ios_5  = 0x037,
  gpio_3_ios_6  = 0x038,
  gpio_3_ios_7  = 0x039,
  gpio_4_ios_0  = 0x03a,
  gpio_4_ios_1  = 0x03b,
  gpio_4_ios_2  = 0x03c,
  gpio_4_ios_3  = 0x03d,
  gpio_4_ios_4  = 0x03e,
  gpio_4_ios_5  = 0x03f,
  uart_0_rx     = 0x040,
  uart_1_rx     = 0x041,
  uart_2_rx     = 0x042,
  spi_0_cipo    = 0x043,
  spi_1_cipo    = 0x044,
  spi_2_cipo    = 0x045,
};

/**
 * Returns the number of sources available for a pin sink (output pin).
 *
 * @param pin_sink The pin sink to query.
 * @returns The number of sources available for the given sink.
 */
static constexpr uint8_t sources_number(PinSink pin_sink) {
  switch (pin_sink) {
    case PinSink::pmod0_2:
    case PinSink::pmod1_2:
      return 5;
    case PinSink::rph_g18:
    case PinSink::rph_g20:
    case PinSink::rph_g21:
    case PinSink::ah_tmpio10:
    case PinSink::ah_tmpio11:
    case PinSink::pmod0_4:
    case PinSink::pmod1_4:
      return 4;
    case PinSink::ser1_tx:
    case PinSink::rph_g0:
    case PinSink::rph_g1:
    case PinSink::rph_g2_sda:
    case PinSink::rph_g3_scl:
    case PinSink::rph_g7:
    case PinSink::rph_g8:
    case PinSink::rph_g10:
    case PinSink::rph_g11:
    case PinSink::rph_g12:
    case PinSink::rph_g13:
    case PinSink::rph_txd0:
    case PinSink::rph_g16:
    case PinSink::rph_g17:
    case PinSink::rph_g19:
    case PinSink::ah_tmpio1:
    case PinSink::ah_tmpio3:
    case PinSink::ah_tmpio5:
    case PinSink::ah_tmpio6:
    case PinSink::ah_tmpio9:
    case PinSink::ah_tmpio13:
    case PinSink::pmod0_1:
    case PinSink::pmod0_3:
    case PinSink::pmod0_8:
    case PinSink::pmod0_9:
    case PinSink::pmod0_10:
    case PinSink::pmod1_1:
    case PinSink::pmod1_3:
    case PinSink::pmod1_8:
    case PinSink::pmod1_9:
    case PinSink::pmod1_10:
      return 3;
    default:
      return 2;
  }
}

/**
 * Returns the number of sources available for a block sink (block input).
 *
 * @param block_sink The block sink to query.
 * @returns The number of sources available for the given sink.
 */
static constexpr uint8_t sources_number(BlockSink block_sink) {
  switch (block_sink) {
    case BlockSink::uart_1_rx:
      return 6;
    case BlockSink::uart_2_rx:
      return 5;
    case BlockSink::spi_1_cipo:
    case BlockSink::spi_2_cipo:
      return 4;
    case BlockSink::spi_0_cipo:
      return 3;
    default:
      return 2;
  }
}

/**
 * A handle to a sink configuration register. This can be used to select
 * the source of the handle's associated sink.
 */
template <typename SinkEnum>
struct Sink {
  CHERI::Capability<volatile uint8_t> reg;
  const SinkEnum sink;

  /**
   * Select a source to connect to the sink.
   *
   * To see the sources available for a given sink see the Sonata system
   * documentation:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux/
   *
   * Note, source 0 disconnects the sink from any source disabling it,
   * and source 1 is the default source for any given sink.
   */
  bool select(uint8_t source) {
    if (source >= sources_number(sink)) {
      Debug::log("{} is outside the range of valid sources, [0-{}), of pin {}.", source, sources_number(sink), sink);
      return false;
    }
    *reg = 1 << source;
    return true;
  }

  /// Disconnect the sink from all available sources.
  void disable() { *reg = 1 << SourceDisabled; }

  /// Reset the sink to it's default source.
  void default_selection() { *reg = 1 << SourceDefault; }
};

namespace {
template <typename SinkEnum>
// This is used by `BlockSinks` and `PinSinks`
// to return a capability to a single sink's configuration register.
inline Sink<SinkEnum> _get_sink(volatile uint8_t *base_register, const SinkEnum sink) {
  CHERI::Capability reg = {base_register + static_cast<ptrdiff_t>(sink)};
  reg.bounds()          = sizeof(uint8_t);
  return Sink<SinkEnum>{reg, sink};
};
}  // namespace

/**
 * A driver for the Sonata system's pin multiplexed output pins.
 *
 * The Sonata system's Pin Multiplexer (pinmux) has two sets of registers: the pin sink
 * registers and the block sink registers. This structure provides access to the
 * pin sinks registers. Pin sinks are output onto the Sonata system's pins, which
 * can be connected to a number of block outputs (their sources). The sources each sink
 * can connect to are limited. See the documentation for the possible sources for
 * a given pin:
 *
 * https://lowrisc.github.io/sonata-system/doc/ip/pinmux/
 */
struct PinSinks : private utils::NoCopyNoMove {
  volatile uint8_t registers[NumPinSinks];

  /// Returns a handle to a pin sink (an output pin).
  Sink<PinSink> get(PinSink sink) volatile { return _get_sink<PinSink>(registers, sink); };
};

/**
 * A driver for the Sonata system's pin multiplexed block inputs.
 *
 * The Sonata system's Pin Multiplexer (pinmux) has two sets of registers: the pin sink
 * registers and the block sink registers. This structure provides access to the
 * block sinks registers. Block sinks are inputs into the Sonata system's devices
 * that can be connected to a number of system input pins (their sources). The sources
 * each sink can connect to are limited. See the documentation for the possible sources
 * for a given pin:
 *
 * https://lowrisc.github.io/sonata-system/doc/ip/pinmux
 */
struct BlockSinks : private utils::NoCopyNoMove {
  volatile uint8_t registers[NumBlockSinks];

  /// Returns a handle to a block sink (a block input).
  Sink<BlockSink> get(BlockSink sink) volatile { return _get_sink<BlockSink>(registers, sink); };
};
}  // namespace SonataPinmux
