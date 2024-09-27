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
 * Documentation source can be found at:
 * https://github.com/lowRISC/sonata-system/blob/6914b30a2769f881a53afb912032cd65a6d14307/doc/ip/pinmux.md
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
    Serial0Transmit        = 0x000,  // ser0_tx,      i.e. P12.4
    Serial1Transmit        = 0x001,  // ser1_tx,      i.e. P12.8
    Rs232Transmit          = 0x002,  // rs232_tx
    I2cSerialClock0        = 0x003,  // scl0,         i.e. J1.4
    I2cSerialData0         = 0x004,  // sda0,         i.e. J1.3
    I2cSerialClock1        = 0x005,  // scl1,         i.e. J7.4
    I2cSerialData1         = 0x006,  // sda1,         i.e. J7.3
    SpiFlashData           = 0x007,  // appspi_d0
    SpiFlashClock          = 0x008,  // appspi_clk
    SpiLcdCopi             = 0x009,  // lcd_copi,     i.e. LCD1.8
    SpiLcdClock            = 0x00a,  // lcd_clk,      i.e. LCD1.9
    SpiEthernetCopi        = 0x00b,  // ethmac_copi,  i.e. U6.31
    SpiEthernetClock       = 0x00c,  // ethmac_sclk,  i.e. U6.28
    RaspberryPiHat27       = 0x00d,  // rph_g0,       i.e. P3.27
    RaspberryPiHat28       = 0x00e,  // rph_g1,       i.e. P3.28
    RaspberryPiHat3        = 0x00f,  // rph_g2_sda,   i.e. P3.3
    RaspberryPiHat5        = 0x010,  // rph_g3_scl,   i.e. P3.5
    RaspberryPiHat7        = 0x011,  // rph_g4,       i.e. P3.7
    RaspberryPiHat29       = 0x012,  // rph_g5,       i.e. P3.29
    RaspberryPiHat31       = 0x013,  // rph_g6,       i.e. P3.31
    RaspberryPiHat26       = 0x014,  // rph_g7_ce1,   i.e. P3.26
    RaspberryPiHat24       = 0x015,  // rph_g8_ce0,   i.e. P3.24
    RaspberryPiHat21       = 0x016,  // rph_g9_cipo,  i.e. P3.21
    RaspberryPiHat19       = 0x017,  // rph_g10_copi, i.e. P3.19
    RaspberryPiHat23       = 0x018,  // rph_g11_sclk, i.e. P3.23
    RaspberryPiHat32       = 0x019,  // rph_g12,      i.e. P3.32
    RaspberryPiHat33       = 0x01a,  // rph_g13,      i.e. P3.33
    RaspberryPiHat8        = 0x01b,  // rph_txd0,     i.e. P3.8
    RaspberryPiHat10       = 0x01c,  // rph_rxd0,     i.e. P3.10
    RaspberryPiHat36       = 0x01d,  // rph_g16_ce2,  i.e. P3.36
    RaspberryPiHat11       = 0x01e,  // rph_g17,      i.e. P3.11
    RaspberryPiHat12       = 0x01f,  // rph_g18,      i.e. P3.12
    RaspberryPiHat35       = 0x020,  // rph_g19_cipo, i.e. P3.35
    RaspberryPiHat38       = 0x021,  // rph_g20_copi, i.e. P3.38
    RaspberryPiHat40       = 0x022,  // rph_g21_sclk, i.e. P3.40
    RaspberryPiHat15       = 0x023,  // rph_g22,      i.e. P3.15
    RaspberryPiHat16       = 0x024,  // rph_g23,      i.e. P3.16
    RaspberryPiHat18       = 0x025,  // rph_g24,      i.e. P3.18
    RaspberryPiHat22       = 0x026,  // rph_g25,      i.e. P3.22
    RaspberryPiHat37       = 0x027,  // rph_g26,      i.e. P3.37
    RaspberryPiHat13       = 0x028,  // rph_g27,      i.e. P3.13
    ArduinoShieldD0        = 0x029,  // ah_tmpio0,    i.e. P1.1
    ArduinoShieldD1        = 0x02a,  // ah_tmpio1,    i.e. P1.2
    ArduinoShieldD2        = 0x02b,  // ah_tmpio2,    i.e. P1.3
    ArduinoShieldD3        = 0x02c,  // ah_tmpio3,    i.e. P1.4
    ArduinoShieldD4        = 0x02d,  // ah_tmpio4,    i.e. P1.5
    ArduinoShieldD5        = 0x02e,  // ah_tmpio5,    i.e. P1.6
    ArduinoShieldD6        = 0x02f,  // ah_tmpio6,    i.e. P1.7
    ArduinoShieldD7        = 0x030,  // ah_tmpio7,    i.e. P1.8
    ArduinoShieldD8        = 0x031,  // ah_tmpio8,    i.e. P4.1
    ArduinoShieldD9        = 0x032,  // ah_tmpio9,    i.e. P4.2
    ArduinoShieldD10       = 0x033,  // ah_tmpio10,   i.e. P4.3
    ArduinoShieldD11       = 0x034,  // ah_tmpio11,   i.e. P4.4
    ArduinoShieldD12       = 0x035,  // ah_tmpio12,   i.e. P4.5
    ArduinoShieldD13       = 0x036,  // ah_tmpio13,   i.e. P4.6
    ArduinoShieldCipo      = 0x037,  // ah_tmpio14,   i.e. P11.1
    ArduinoShieldSck       = 0x038,  // ah_tmpio15,   i.e. P11.3
    ArduinoShieldIo        = 0x039,  // ah_tmpio16,   i.e. P11.5
    ArduinoShieldCopi      = 0x03a,  // ah_tmpio17,   i.e. P11.4
    MikroBusSpiClock       = 0x03b,  // mb2,          i.e. P6.4
    MikroBusSpiCopi        = 0x03c,  // mb4,          i.e. P6.6
    MikroBusI2cSerialData  = 0x03d,  // mb5,          i.e. P7.6
    MikroBusI2cSerialClock = 0x03e,  // mb6,          i.e. P7.5
    MikroBusUartTransmit   = 0x03f,  // mb7,          i.e. P7.4
    Pmod0Io1               = 0x040,  // pmod0_0,      i.e. PMOD0.1
    Pmod0Io2               = 0x041,  // pmod0_1,      i.e. PMOD0.2
    Pmod0Io3               = 0x042,  // pmod0_2,      i.e. PMOD0.3
    Pmod0Io4               = 0x043,  // pmod0_3,      i.e. PMOD0.4
    Pmod0Io7               = 0x044,  // pmod0_4,      i.e. PMOD0.7
    Pmod0Io8               = 0x045,  // pmod0_5,      i.e. PMOD0.8
    Pmod0Io9               = 0x046,  // pmod0_6,      i.e. PMOD0.9
    Pmod0Io10              = 0x047,  // pmod0_7,      i.e. PMOD0.10
    Pmod1Io1               = 0x048,  // pmod1_0,      i.e. PMOD1.1
    Pmod1Io2               = 0x049,  // pmod1_1,      i.e. PMOD1.2
    Pmod1Io3               = 0x04a,  // pmod1_2,      i.e. PMOD1.3
    Pmod1Io4               = 0x04b,  // pmod1_3,      i.e. PMOD1.4
    Pmod1Io7               = 0x04c,  // pmod1_4,      i.e. PMOD1.7
    Pmod1Io8               = 0x04d,  // pmod1_5,      i.e. PMOD1.8
    Pmod1Io9               = 0x04e,  // pmod1_6,      i.e. PMOD1.9
    Pmod1Io10              = 0x04f,  // pmod1_7,      i.e. PMOD1.10
  };

  /**
   * The Block Inputs defined for the Sonata board. These block inputs can
   * be multiplexed, meaning that they can be changed to take the input of
   * different pins (or be disabled). The pin inputs that can be selected
   * are limited, and vary on a per-block-input basis.
   *
   * Documentation source:
   * https://lowrisc.github.io/sonata-system/doc/ip/pinmux.html
   * */
  enum class BlockInput : uint16_t {
    UartReceive0        = 0x800,  // uart_rx_o_0
    UartReceive1        = 0x801,  // uart_rx_o_1
    UartReceive2        = 0x802,  // uart_rx_o_2
    UartReceive3        = 0x803,  // uart_rx_o_3
    UartReceive4        = 0x804,  // uart_rx_o_4
    SpiReceive0         = 0x805,  // spi_rx_o_0
    SpiReceive1         = 0x806,  // spi_rx_o_1
    SpiReceive2         = 0x807,  // spi_rx_o_2
    SpiReceive3         = 0x808,  // spi_rx_o_3
    SpiReceive4         = 0x809,  // spi_rx_o_4
    RaspberryPiGpio0    = 0x80a,  // gpio_ios_o_0_0
    ArduinoShieldGpio0  = 0x80b,  // gpio_ios_o_1_0
    PmodGpio0           = 0x80c,  // gpio_ios_o_2_0
    RaspberryPiGpio1    = 0x80d,  // gpio_ios_o_0_1
    ArduinoShieldGpio1  = 0x80e,  // gpio_ios_o_1_1
    PmodGpio1           = 0x80f,  // gpio_ios_o_2_1
    RaspberryPiGpio2    = 0x810,  // gpio_ios_o_0_2
    ArduinoShieldGpio2  = 0x811,  // gpio_ios_o_1_2
    PmodGpio2           = 0x812,  // gpio_ios_o_2_2
    RaspberryPiGpio3    = 0x813,  // gpio_ios_o_0_3
    ArduinoShieldGpio3  = 0x814,  // gpio_ios_o_1_3
    PmodGpio3           = 0x815,  // gpio_ios_o_2_3
    RaspberryPiGpio4    = 0x816,  // gpio_ios_o_0_4
    ArduinoShieldGpio4  = 0x817,  // gpio_ios_o_1_4
    PmodGpio4           = 0x818,  // gpio_ios_o_2_4
    RaspberryPiGpio5    = 0x819,  // gpio_ios_o_0_5
    ArduinoShieldGpio5  = 0x81a,  // gpio_ios_o_1_5
    PmodGpio5           = 0x81b,  // gpio_ios_o_2_5
    RaspberryPiGpio6    = 0x81c,  // gpio_ios_o_0_6
    ArduinoShieldGpio6  = 0x81d,  // gpio_ios_o_1_6
    PmodGpio6           = 0x81e,  // gpio_ios_o_2_6
    RaspberryPiGpio7    = 0x81f,  // gpio_ios_o_0_7
    ArduinoShieldGpio7  = 0x820,  // gpio_ios_o_1_7
    PmodGpio7           = 0x821,  // gpio_ios_o_2_7
    RaspberryPiGpio8    = 0x822,  // gpio_ios_o_0_8
    ArduinoShieldGpio8  = 0x823,  // gpio_ios_o_1_8
    PmodGpio8           = 0x824,  // gpio_ios_o_2_8
    RaspberryPiGpio9    = 0x825,  // gpio_ios_o_0_9
    ArduinoShieldGpio9  = 0x826,  // gpio_ios_o_1_9
    PmodGpio9           = 0x827,  // gpio_ios_o_2_9
    RaspberryPiGpio10   = 0x828,  // gpio_ios_o_0_10
    ArduinoShieldGpio10 = 0x829,  // gpio_ios_o_1_10
    PmodGpio10          = 0x82a,  // gpio_ios_o_2_10
    RaspberryPiGpio11   = 0x82b,  // gpio_ios_o_0_11
    ArduinoShieldGpio11 = 0x82c,  // gpio_ios_o_1_11
    PmodGpio11          = 0x82d,  // gpio_ios_o_2_11
    RaspberryPiGpio12   = 0x82e,  // gpio_ios_o_0_12
    ArduinoShieldGpio12 = 0x82f,  // gpio_ios_o_1_12
    PmodGpio12          = 0x830,  // gpio_ios_o_2_12
    RaspberryPiGpio13   = 0x831,  // gpio_ios_o_0_13
    ArduinoShieldGpio13 = 0x832,  // gpio_ios_o_1_13
    PmodGpio13          = 0x833,  // gpio_ios_o_2_13
    RaspberryPiGpio14   = 0x834,  // gpio_ios_o_0_14
    ArduinoShieldGpio14 = 0x835,  // gpio_ios_o_1_14
    PmodGpio14          = 0x836,  // gpio_ios_o_2_14
    RaspberryPiGpio15   = 0x837,  // gpio_ios_o_0_15
    ArduinoShieldGpio15 = 0x838,  // gpio_ios_o_1_15
    PmodGpio15          = 0x839,  // gpio_ios_o_2_15
    RaspberryPiGpio16   = 0x83a,  // gpio_ios_o_0_16
    ArduinoShieldGpio16 = 0x83b,  // gpio_ios_o_1_16
    PmodGpio16          = 0x83c,  // gpio_ios_o_2_16
    RaspberryPiGpio17   = 0x83d,  // gpio_ios_o_0_17
    ArduinoShieldGpio17 = 0x83e,  // gpio_ios_o_1_17
    PmodGpio17          = 0x83f,  // gpio_ios_o_2_17
    RaspberryPiGpio18   = 0x840,  // gpio_ios_o_0_18
    ArduinoShieldGpio18 = 0x841,  // gpio_ios_o_1_18
    PmodGpio18          = 0x842,  // gpio_ios_o_2_18
    RaspberryPiGpio19   = 0x843,  // gpio_ios_o_0_19
    ArduinoShieldGpio19 = 0x844,  // gpio_ios_o_1_19
    PmodGpio19          = 0x845,  // gpio_ios_o_2_19
    RaspberryPiGpio20   = 0x846,  // gpio_ios_o_0_20
    ArduinoShieldGpio20 = 0x847,  // gpio_ios_o_1_20
    PmodGpio20          = 0x848,  // gpio_ios_o_2_20
    RaspberryPiGpio21   = 0x849,  // gpio_ios_o_0_21
    ArduinoShieldGpio21 = 0x84a,  // gpio_ios_o_1_21
    PmodGpio21          = 0x84b,  // gpio_ios_o_2_21
    RaspberryPiGpio22   = 0x84c,  // gpio_ios_o_0_22
    ArduinoShieldGpio22 = 0x84d,  // gpio_ios_o_1_22
    PmodGpio22          = 0x84e,  // gpio_ios_o_2_22
    RaspberryPiGpio23   = 0x84f,  // gpio_ios_o_0_23
    ArduinoShieldGpio23 = 0x850,  // gpio_ios_o_1_23
    PmodGpio23          = 0x851,  // gpio_ios_o_2_23
    RaspberryPiGpio24   = 0x852,  // gpio_ios_o_0_24
    ArduinoShieldGpio24 = 0x853,  // gpio_ios_o_1_24
    PmodGpio24          = 0x854,  // gpio_ios_o_2_24
    RaspberryPiGpio25   = 0x855,  // gpio_ios_o_0_25
    ArduinoShieldGpio25 = 0x856,  // gpio_ios_o_1_25
    PmodGpio25          = 0x857,  // gpio_ios_o_2_25
    RaspberryPiGpio26   = 0x858,  // gpio_ios_o_0_26
    ArduinoShieldGpio26 = 0x859,  // gpio_ios_o_1_26
    PmodGpio26          = 0x85a,  // gpio_ios_o_2_26
    RaspberryPiGpio27   = 0x85b,  // gpio_ios_o_0_27
    ArduinoShieldGpio27 = 0x85c,  // gpio_ios_o_1_27
    PmodGpio27          = 0x85d,  // gpio_ios_o_2_27
    RaspberryPiGpio28   = 0x85e,  // gpio_ios_o_0_28
    ArduinoShieldGpio28 = 0x85f,  // gpio_ios_o_1_28
    PmodGpio28          = 0x860,  // gpio_ios_o_2_28
    RaspberryPiGpio29   = 0x861,  // gpio_ios_o_0_29
    ArduinoShieldGpio29 = 0x862,  // gpio_ios_o_1_29
    PmodGpio29          = 0x863,  // gpio_ios_o_2_29
    RaspberryPiGpio30   = 0x864,  // gpio_ios_o_0_30
    ArduinoShieldGpio30 = 0x865,  // gpio_ios_o_1_30
    PmodGpio30          = 0x866,  // gpio_ios_o_2_30
    RaspberryPiGpio31   = 0x867,  // gpio_ios_o_0_31
    ArduinoShieldGpio31 = 0x868,  // gpio_ios_o_1_31
    PmodGpio31          = 0x869,  // gpio_ios_o_2_31
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
      case OutputPin::RaspberryPiHat27:
      case OutputPin::RaspberryPiHat28:
      case OutputPin::RaspberryPiHat3:
      case OutputPin::RaspberryPiHat5:
      case OutputPin::RaspberryPiHat19:
      case OutputPin::RaspberryPiHat23:
      case OutputPin::RaspberryPiHat8:
      case OutputPin::RaspberryPiHat38:
      case OutputPin::RaspberryPiHat40:
      case OutputPin::ArduinoShieldD11:
      case OutputPin::ArduinoShieldD13:
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
      case BlockInput::SpiReceive3:
      case BlockInput::SpiReceive4:
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
