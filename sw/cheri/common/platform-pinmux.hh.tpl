/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

${"#"}pragma once
${"#"}include <debug.hh>
${"#"}include <stdint.h>
${"#"}include <utils.hh>

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
% for output_idx, (pin, _, _) in enumerate(output_pins):
    ${pin.doc_name.replace("[","_").replace("]","")} = ${f"{output_idx:#0{5}x}"},
% endfor
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
% for input_idx, (block_io, possible_pins, num_options) in enumerate(output_block_ios):
    ${block_io.doc_name.replace("[","_").replace(".","_").replace("]","")} = ${f"{(0x800+input_idx):#0{5}x}"}, 
% endfor
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
<%
    prev_num_options = 0
%>
% for (pin, _, num_options) in filter(lambda pin: pin[2] > 2, sorted(output_pins, key=lambda pin: pin[2], reverse=True)):
    % if prev_num_options != 0 and prev_num_options != num_options:
        return ${prev_num_options};
    % endif
    <%
        prev_num_options = num_options 
    %>      case OutputPin::${pin.doc_name.replace("[","_").replace("]","")}:
% endfor
% if prev_num_options != 0:
        return ${prev_num_options};
% endif
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
<%
    prev_num_options = 0
%>
% for (block_io, _, num_options) in filter(lambda block: block[2] > 2, sorted(output_block_ios, key=lambda block: block[2], reverse=True)):
    % if prev_num_options != 0 and prev_num_options != num_options:
        return ${prev_num_options};
    % endif
    <%
        prev_num_options = num_options 
    %>      case BlockInput::${block_io.doc_name.replace("[","_").replace(".","_").replace("]","")}:
% endfor
% if prev_num_options != 0:
        return ${prev_num_options};
% endif
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
