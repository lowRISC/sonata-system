/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <debug.hh>
#include <stdint.h>
#include <utils.hh>

/**
 * A driver for Sonata's Pin Multiplexer (Pinmux).
 *
 * This driver can be used to select which block output is placed on a given
 * output pin, and to select which input pin is provided to a given block
 * input. Sonata's pinmux only allows certain selections per output pin / block
 * input, which this driver describes.
 *
 * Rendered documentation is served from:
 * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
 */
class SonataPinmux : private utils::NoCopyNoMove {
  /**
   * Flag to set when debugging the driver for UART log messages.
   */
  static constexpr bool DebugDriver = false;

  /**
   * Helper for conditional debug logs and assertions.
   */
  using Debug = ConditionalDebug<DebugDriver, "Pinmux">;

  /**
   * A pointer/capability to the Pin Multiplexer's registers, where
   * each sequential byte potentially corresponds to some mapped pinmux
   * selector used to select the pin configuration.
   */
  volatile uint8_t *registers;

 public:
  /**
   * The Output Pins defined for the Sonata board. These output pins can be
   * multiplexed, meaning that they can be changed to output the outputs
   * of different blocks (or disabled). The block outputs that can be
   * selected are limited, and vary on a per-pin basis.
   *
   * Documentation sources:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   * https://github.com/lowRISC/sonata-system/blob/4b72d8c07c727846c6ccb27754352388f3b2ac9a/data/pins_sonata.xdc
   * https://github.com/newaetech/sonata-pcb/blob/649b11c2fb758f798966605a07a8b6b68dd434e9/sonata-schematics-r09.pdf
   */
  enum class OutputPin : uint16_t {
    ser0_tx      = 0x000,
    ser1_tx      = 0x001,
    rs232_tx     = 0x002,
    scl0         = 0x003,
    sda0         = 0x004,
    scl1         = 0x005,
    sda1         = 0x006,
    appspi_d0    = 0x007,
    appspi_clk   = 0x008,
    appspi_cs    = 0x009,
    lcd_copi     = 0x00a,
    lcd_clk      = 0x00b,
    lcd_cs       = 0x00c,
    ethmac_copi  = 0x00d,
    ethmac_sclk  = 0x00e,
    ethmac_cs    = 0x00f,
    rph_g0       = 0x010,
    rph_g1       = 0x011,
    rph_g2_sda   = 0x012,
    rph_g3_scl   = 0x013,
    rph_g4       = 0x014,
    rph_g5       = 0x015,
    rph_g6       = 0x016,
    rph_g7_ce1   = 0x017,
    rph_g8_ce0   = 0x018,
    rph_g9_cipo  = 0x019,
    rph_g10_copi = 0x01a,
    rph_g11_sclk = 0x01b,
    rph_g12      = 0x01c,
    rph_g13      = 0x01d,
    rph_txd0     = 0x01e,
    rph_rxd0     = 0x01f,
    rph_g16_ce2  = 0x020,
    rph_g17      = 0x021,
    rph_g18      = 0x022,
    rph_g19_cipo = 0x023,
    rph_g20_copi = 0x024,
    rph_g21_sclk = 0x025,
    rph_g22      = 0x026,
    rph_g23      = 0x027,
    rph_g24      = 0x028,
    rph_g25      = 0x029,
    rph_g26      = 0x02a,
    rph_g27      = 0x02b,
    ah_tmpio0    = 0x02c,
    ah_tmpio1    = 0x02d,
    ah_tmpio2    = 0x02e,
    ah_tmpio3    = 0x02f,
    ah_tmpio4    = 0x030,
    ah_tmpio5    = 0x031,
    ah_tmpio6    = 0x032,
    ah_tmpio7    = 0x033,
    ah_tmpio8    = 0x034,
    ah_tmpio9    = 0x035,
    ah_tmpio10   = 0x036,
    ah_tmpio11   = 0x037,
    ah_tmpio12   = 0x038,
    ah_tmpio13   = 0x039,
    mb1          = 0x03a,
    mb2          = 0x03b,
    mb4          = 0x03c,
    mb5          = 0x03d,
    mb6          = 0x03e,
    mb7          = 0x03f,
    mb10         = 0x040,
    pmod0_1      = 0x041,
    pmod0_2      = 0x042,
    pmod0_3      = 0x043,
    pmod0_4      = 0x044,
    pmod0_5      = 0x045,
    pmod0_6      = 0x046,
    pmod0_7      = 0x047,
    pmod0_8      = 0x048,
    pmod1_1      = 0x049,
    pmod1_2      = 0x04a,
    pmod1_3      = 0x04b,
    pmod1_4      = 0x04c,
    pmod1_5      = 0x04d,
    pmod1_6      = 0x04e,
    pmod1_7      = 0x04f,
    pmod1_8      = 0x050,
    microsd_clk  = 0x051,
    microsd_cmd  = 0x052,
    microsd_dat3 = 0x053,
  };

  /**
   * The Block Inputs defined for the Sonata board. These block inputs can
   * be multiplexed, meaning that they can be changed to take the input of
   * different pins (or be disabled). The pin inputs that can be selected
   * are limited, and vary on a per-block-input basis.
   *
   * For reference:
   *   gpio_0 = Raspberry Pi
   *   gpio_1 = ArduinoShield
   *   gpio_2 = Pmod
   *
   * Documentation source:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   * */
  enum class BlockInput : uint16_t {
    gpio_0_ios_0  = 0x800,
    gpio_0_ios_1  = 0x801,
    gpio_0_ios_2  = 0x802,
    gpio_0_ios_3  = 0x803,
    gpio_0_ios_4  = 0x804,
    gpio_0_ios_5  = 0x805,
    gpio_0_ios_6  = 0x806,
    gpio_0_ios_7  = 0x807,
    gpio_0_ios_8  = 0x808,
    gpio_0_ios_9  = 0x809,
    gpio_0_ios_10 = 0x80a,
    gpio_0_ios_11 = 0x80b,
    gpio_0_ios_12 = 0x80c,
    gpio_0_ios_13 = 0x80d,
    gpio_0_ios_14 = 0x80e,
    gpio_0_ios_15 = 0x80f,
    gpio_0_ios_16 = 0x810,
    gpio_0_ios_17 = 0x811,
    gpio_0_ios_18 = 0x812,
    gpio_0_ios_19 = 0x813,
    gpio_0_ios_20 = 0x814,
    gpio_0_ios_21 = 0x815,
    gpio_0_ios_22 = 0x816,
    gpio_0_ios_23 = 0x817,
    gpio_0_ios_24 = 0x818,
    gpio_0_ios_25 = 0x819,
    gpio_0_ios_26 = 0x81a,
    gpio_0_ios_27 = 0x81b,
    gpio_0_ios_28 = 0x81c,
    gpio_0_ios_29 = 0x81d,
    gpio_0_ios_30 = 0x81e,
    gpio_0_ios_31 = 0x81f,
    gpio_1_ios_0  = 0x820,
    gpio_1_ios_1  = 0x821,
    gpio_1_ios_2  = 0x822,
    gpio_1_ios_3  = 0x823,
    gpio_1_ios_4  = 0x824,
    gpio_1_ios_5  = 0x825,
    gpio_1_ios_6  = 0x826,
    gpio_1_ios_7  = 0x827,
    gpio_1_ios_8  = 0x828,
    gpio_1_ios_9  = 0x829,
    gpio_1_ios_10 = 0x82a,
    gpio_1_ios_11 = 0x82b,
    gpio_1_ios_12 = 0x82c,
    gpio_1_ios_13 = 0x82d,
    gpio_1_ios_14 = 0x82e,
    gpio_1_ios_15 = 0x82f,
    gpio_1_ios_16 = 0x830,
    gpio_1_ios_17 = 0x831,
    gpio_1_ios_18 = 0x832,
    gpio_1_ios_19 = 0x833,
    gpio_1_ios_20 = 0x834,
    gpio_1_ios_21 = 0x835,
    gpio_1_ios_22 = 0x836,
    gpio_1_ios_23 = 0x837,
    gpio_1_ios_24 = 0x838,
    gpio_1_ios_25 = 0x839,
    gpio_1_ios_26 = 0x83a,
    gpio_1_ios_27 = 0x83b,
    gpio_1_ios_28 = 0x83c,
    gpio_1_ios_29 = 0x83d,
    gpio_1_ios_30 = 0x83e,
    gpio_1_ios_31 = 0x83f,
    gpio_2_ios_0  = 0x840,
    gpio_2_ios_1  = 0x841,
    gpio_2_ios_2  = 0x842,
    gpio_2_ios_3  = 0x843,
    gpio_2_ios_4  = 0x844,
    gpio_2_ios_5  = 0x845,
    gpio_2_ios_6  = 0x846,
    gpio_2_ios_7  = 0x847,
    gpio_2_ios_8  = 0x848,
    gpio_2_ios_9  = 0x849,
    gpio_2_ios_10 = 0x84a,
    gpio_2_ios_11 = 0x84b,
    gpio_2_ios_12 = 0x84c,
    gpio_2_ios_13 = 0x84d,
    gpio_2_ios_14 = 0x84e,
    gpio_2_ios_15 = 0x84f,
    gpio_2_ios_16 = 0x850,
    gpio_2_ios_17 = 0x851,
    gpio_2_ios_18 = 0x852,
    gpio_2_ios_19 = 0x853,
    gpio_2_ios_20 = 0x854,
    gpio_2_ios_21 = 0x855,
    gpio_2_ios_22 = 0x856,
    gpio_2_ios_23 = 0x857,
    gpio_2_ios_24 = 0x858,
    gpio_2_ios_25 = 0x859,
    gpio_2_ios_26 = 0x85a,
    gpio_2_ios_27 = 0x85b,
    gpio_2_ios_28 = 0x85c,
    gpio_2_ios_29 = 0x85d,
    gpio_2_ios_30 = 0x85e,
    gpio_2_ios_31 = 0x85f,
    gpio_3_ios_0  = 0x860,
    gpio_3_ios_1  = 0x861,
    gpio_3_ios_2  = 0x862,
    gpio_3_ios_3  = 0x863,
    gpio_3_ios_4  = 0x864,
    gpio_3_ios_5  = 0x865,
    gpio_3_ios_6  = 0x866,
    gpio_3_ios_7  = 0x867,
    gpio_3_ios_8  = 0x868,
    gpio_3_ios_9  = 0x869,
    gpio_3_ios_10 = 0x86a,
    gpio_3_ios_11 = 0x86b,
    gpio_3_ios_12 = 0x86c,
    gpio_3_ios_13 = 0x86d,
    gpio_3_ios_14 = 0x86e,
    gpio_3_ios_15 = 0x86f,
    gpio_3_ios_16 = 0x870,
    gpio_3_ios_17 = 0x871,
    gpio_3_ios_18 = 0x872,
    gpio_3_ios_19 = 0x873,
    gpio_3_ios_20 = 0x874,
    gpio_3_ios_21 = 0x875,
    gpio_3_ios_22 = 0x876,
    gpio_3_ios_23 = 0x877,
    gpio_3_ios_24 = 0x878,
    gpio_3_ios_25 = 0x879,
    gpio_3_ios_26 = 0x87a,
    gpio_3_ios_27 = 0x87b,
    gpio_3_ios_28 = 0x87c,
    gpio_3_ios_29 = 0x87d,
    gpio_3_ios_30 = 0x87e,
    gpio_3_ios_31 = 0x87f,
    uart_0_rx     = 0x880,
    uart_1_rx     = 0x881,
    uart_2_rx     = 0x882,
    uart_3_rx     = 0x883,
    uart_4_rx     = 0x884,
    spi_0_rx      = 0x885,
    spi_1_rx      = 0x886,
    spi_2_rx      = 0x887,
    spi_3_rx      = 0x888,
    spi_4_rx      = 0x889,
  };

  /**
   * A helper function that returns the number of block outputs that can be
   * selected from in the pin multiplexer for a given output pin. This will
   * always be at least 2, as option 0 represents 'OFF' i.e. no connection,
   * and option 1 represents the default connection.
   *
   * @param output_pin The output pin to query
   * @returns The number of selections available for that output pin
   *
   * The meanings of these selections can be found in the documentation:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   */
  static constexpr uint8_t output_pin_options(OutputPin output_pin) {
    switch (output_pin) {
      case OutputPin::pmod0_2:
      case OutputPin::pmod0_4:
      case OutputPin::pmod1_2:
      case OutputPin::pmod1_4:
        return 4;
      case OutputPin::rph_g0:
      case OutputPin::rph_g1:
      case OutputPin::rph_g2_sda:
      case OutputPin::rph_g3_scl:
      case OutputPin::rph_g7_ce1:
      case OutputPin::rph_g8_ce0:
      case OutputPin::rph_g10_copi:
      case OutputPin::rph_g11_sclk:
      case OutputPin::rph_txd0:
      case OutputPin::rph_g16_ce2:
      case OutputPin::rph_g17:
      case OutputPin::rph_g18:
      case OutputPin::rph_g20_copi:
      case OutputPin::rph_g21_sclk:
      case OutputPin::ah_tmpio1:
      case OutputPin::ah_tmpio10:
      case OutputPin::ah_tmpio11:
      case OutputPin::ah_tmpio13:
      case OutputPin::pmod0_1:
      case OutputPin::pmod0_3:
      case OutputPin::pmod1_1:
      case OutputPin::pmod1_3:
        return 3;
      default:
        return 2;
    }
  }

  /**
   * A helper function that returns the number of input pins that can be
   * selected from in the pin multiplexer for a given block input. This will
   * always be at least 2, as option 0 represents 'OFF' i.e. no connection,
   * and option 1 represents the default connection.
   *
   * @param block_input The block input to query.
   * @returns The number of selections available for that block input.
   *
   * The meanings of these selections can be found in the documentation:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   */
  static constexpr uint8_t block_input_options(BlockInput block_input) {
    switch (block_input) {
      case BlockInput::spi_3_rx:
        return 5;
      case BlockInput::uart_3_rx:
        return 4;
      case BlockInput::uart_2_rx:
      case BlockInput::spi_2_rx:
      case BlockInput::spi_4_rx:
        return 3;
      default:
        return 2;
    }
  }

  /**
   * For a given output pin, selects a given block output to use for that pin
   * via the pin multiplexer.
   *
   * @param output_pin The output pin to pinmux.
   * @param option The option to select for that pin. This value should be
   * less than the value returned by `output_pin_options` for the given pin.
   *
   * The meanings of these selections can be found in the documentation:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   */
  bool output_pin_select(OutputPin output_pin, uint8_t option) {
    if (option >= output_pin_options(output_pin)) {
      Debug::log("Selected option is not valid for this pin.");
      return false;
    }
    uint16_t registerOffset   = static_cast<uint16_t>(output_pin);
    registers[registerOffset] = (1 << option);
    return true;
  }

  /**
   * For a given block input, selects a pin to use for that input via the pin
   * multiplexer.
   *
   * @param block_input The block input to pinmux.
   * @param option The option to select for that block input. This value
   * should be less than the value returned by `block_input_options` for the
   * given block input.
   *
   * The meanings of these selections can be found in the documentation:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   */
  bool block_input_select(BlockInput block_input, uint8_t option) {
    if (option >= block_input_options(block_input)) {
      Debug::log("Selected option is not valid for this block.");
      return false;
    }
    uint16_t registerOffset   = static_cast<uint16_t>(block_input);
    registers[registerOffset] = (1 << option);
    return true;
  }

  /**
   * A constructor for the SonataPinmux driver, which takes a bounded
   * capability to the pinmux registers. This should be replaced with
   * an appropriate `MMIO_CAPABILITY` call in the version of the driver
   * that runs in CHERIoT RTOS, rather than baremetal.
   */
  SonataPinmux(volatile uint8_t *registers) : registers(registers) {}
};
