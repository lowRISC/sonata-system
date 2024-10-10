/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pinmux_checker.hh"
#include "../common/timer-utils.hh"

/**
 * Sets the value of some GPIO output pin. This change will only be visible
 * if the pin has its corresponding `output_enable` bit set to 1, so that the
 * pin is in output mode.
 */
inline void set_gpio_output(Capability<volatile SonataGpioFull> gpio, GpioPin pin, bool value) {
  const uint32_t bit_mask    = (1 << pin.bit);
  const uint32_t write_value = ((value ? 1 : 0) << pin.bit);
  const uint8_t instance     = static_cast<uint8_t>(pin.instance);
  uint32_t output            = gpio->instances[instance].output;
  output &= ~bit_mask;
  output |= write_value;
  gpio->instances[instance].output = output;
}

/**
 * Sets the output enable value of some GPIO pin. Setting this to 1 places the
 * pin in output mode, and to 0 places it in input mode.
 */
inline void set_gpio_output_enable(Capability<volatile SonataGpioFull> gpio, GpioPin pin, bool value) {
  const uint32_t bit_mask    = (1 << pin.bit);
  const uint32_t write_value = ((value ? 1 : 0) << pin.bit);
  const uint8_t instance     = static_cast<uint8_t>(pin.instance);
  uint32_t output_enable     = gpio->instances[instance].output_enable;
  output_enable &= ~bit_mask;
  output_enable |= write_value;
  gpio->instances[instance].output_enable = output_enable;
}

/**
 * Get the value of some GPIO input pin. This will only be visible if the pin
 * has its corresponding `output_enable` bit set to 0, so that the pin is in
 * input mode.
 */
inline bool get_gpio_input(Capability<volatile SonataGpioFull> gpio, GpioPin pin) {
  const uint32_t bit_mask = (1 << pin.bit);
  const uint8_t instance  = static_cast<uint8_t>(pin.instance);
  return (gpio->instances[instance].input & bit_mask) > 0;
}

/**
 * Tests that the given UART appears to be working as expected, by sending and
 * receiving some data using the given UART block. To pass, this test expects
 * the Tx and the Rx of the specified UART to be manually connected on the
 * board, so that data that is written can be read back by the board.
 *
 * @param read_timeout_msec The timeout in milliseconds provided to read each
 * individual byte sent over UART. Any timeout causes immediate failure.
 * @param test_length The number of random bytes to try sending over UART.
 *
 * Returns true if the test passed, or false if it failed.
 */
static bool uart_send_receive_test(ds::xoroshiro::P32R8 &prng, UartPtr uart, uint32_t read_timeout_msec,
                                   uint32_t test_length) {
  constexpr uint8_t UartStatusRegTxIdle = (1 << 2);
  constexpr uint8_t UartStatusRegRxIdle = (1 << 4);

  // Re-initialise the UART and clear its FIFOs.
  uart->init();
  uart->fifos_clear();
  uart->parity();
  reset_mcycle();

  // Wait for the UART TX FIFO to be empty and all bits to be transmitted,
  // and the RX to be idle.
  while ((uart->status & UartStatusRegTxIdle) == 0 || (uart->status & UartStatusRegRxIdle) == 0) {
    asm volatile("");
  }
  uart->fifos_clear();

  // Send and receive a test message, character by character
  for (uint32_t i = 0; i < test_length; ++i) {
    const uint8_t test_byte = prng();
    uart->blocking_write(test_byte);

    // Try to read the character back within the given timeout.
    const uint32_t start_mcycle   = get_mcycle();
    const uint32_t cycles         = read_timeout_msec * CyclesPerMillisecond;
    const uint32_t timeout_mcycle = start_mcycle + cycles;
    while (!uart->can_read() && get_mcycle() < timeout_mcycle) {
      asm volatile("");
    }
    if (!uart->can_read() || uart->blocking_read() != test_byte) {
      // On timeout or an incorrect read, fail immediately.
      return false;
    }
  }

  return true;
}

/**
 * Tests that the given GPIO pins appear to be working as expected, by sending
 * and receiving some data using the specified output and input GPIO pins. To
 * pass, this test expects that the specified input and output GPIO pins are
 * manually connected on the board, so that GPIO output can be written and
 * read back by the board.
 *
 * @param wait_msec The time in milliseconds to wait for the GPIO write signal
 * to be transmitted before reading.
 * @param test_length The number of individual write/reads to test on the given
 * GPIO connection.
 *
 * Returns true if the test passed, or false if it failed.
 */
static bool gpio_write_read_test(Capability<volatile SonataGpioFull> gpio, GpioPin output_pin, GpioPin input_pin,
                                 uint32_t wait_msec, uint32_t test_length) {
  bool gpio_high = false;
  for (uint32_t i = 0; i < test_length; i++) {
    // For each test iteration, invert the GPIO signal
    gpio_high = !gpio_high;

    // Write this inverted signal on the configured output GPIO
    set_gpio_output(gpio, output_pin, gpio_high);

    // Wait a short time for the transmission
    const uint32_t cycles     = wait_msec * CyclesPerMillisecond;
    const uint32_t end_mcycle = get_mcycle() + cycles;
    while (get_mcycle() < end_mcycle) {
      asm volatile("");
    }

    // Read the GPIO value back
    if (get_gpio_input(gpio, input_pin) != gpio_high) {
      return false;
    }
  }
  return true;
}

/**
 * Resets an I2C controller to acknowledge and disable any Controller Halt
 * events. When the I2C device is disconnected from Pinmux and tested,
 * the controller will continue to halt as it will not be able to see
 * changes it makes. As such, we disable the I2C block and clear its
 * controller events, to get it back into a normal state.
 */
static void reset_i2c_controller(I2cPtr i2c) {
  i2c->control = i2c->control & ~(i2c->ControlEnableHost | i2c->ControlEnableTarget);
  if (i2c->interrupt_is_asserted(OpenTitanI2cInterrupt::ControllerHalt)) {
    i2c->reset_controller_events();
  }
}

/**
 * Tests that the given I2C block is working as expected, by sending and
 * receiving some data via I2C to read the JEDEC/Manufacturer ID of a connected
 * BH1745 sensor device, and checking it against known values. This test expects
 * that the I2C SDA and SCL pins corresponding to the the given I2C blocks
 * are connected to a BH1745, with the I2C pins pulled up.
 *
 * Returns true if the test passed, or false if it failed.
 */
static bool i2c_read_bh1745_id_test(I2cPtr i2c) {
  constexpr uint8_t Bh1745DeviceAddr             = 0x38;
  constexpr uint8_t Bh1745ReadManufacturerIDAddr = 0x92;
  constexpr uint8_t ExpectedBh1745ManufacturerID = 0xE0;

  // Setup the I2C bus, configuring it in host mode with speed of 100
  reset_i2c_controller(i2c);
  i2c->reset_fifos();
  i2c->host_mode_set();
  i2c->speed_set(100);

  /// Send the I2C device address
  const uint8_t addr[] = {Bh1745ReadManufacturerIDAddr};
  if (!i2c->blocking_write(Bh1745DeviceAddr, addr, sizeof(addr), true)) {
    return false;
  }

  // Read from the `MANUFACTURER ID` register of the BH1745 sensor,
  // and check it matches the expected value.
  uint8_t data[1] = {0xFF};
  return i2c->blocking_read(Bh1745DeviceAddr, data, 1U) && (data[0] == ExpectedBh1745ManufacturerID);
}

/**
 * Tests that the given I2C block is working as expected, by sending and
 * receiving some data via I2C to read the Device/Part ID of a connected
 * PMOD Colour device, and checking it against known values. This test
 * expects that the I2C SDA and SCL pins corresponding to the given I2C
 * blocks are connected to a PMOD Colour device, with the I2C pins pulled up.
 *
 * Returns true if the test passed, or false if it failed.
 */
static bool i2c_read_pmod_colour_id_test(I2cPtr i2c, Log &log) {
  constexpr uint8_t PmodColourDeviceAddr           = 0x29;
  constexpr uint8_t PmodColourReadDeviceIdAddr     = 0x12;
  constexpr uint8_t ExpectedPmodColourDeviceIdRevA = 0x18;
  constexpr uint8_t ExpectedPmodColourDeviceIdRevB = 0x12;

  // Setup the I2C bus, configuring it in host mode with speed of 100.
  reset_i2c_controller(i2c);
  i2c->reset_fifos();
  i2c->host_mode_set();
  i2c->speed_set(100);

  /// Send the I2C device address
  const uint8_t addr[] = {PmodColourReadDeviceIdAddr};
  if (!i2c->blocking_write(PmodColourDeviceAddr, addr, sizeof(addr), true)) {
    return false;
  }

  // Read from the `ID` register of the PMOD Colour sensor, and check it
  // matches the expected value.
  uint8_t data[1] = {0xFF};
  return i2c->blocking_read(PmodColourDeviceAddr, data, 1U) &&
         ((data[0] == ExpectedPmodColourDeviceIdRevA) || (data[0] == ExpectedPmodColourDeviceIdRevB));
}

/**
 * Tests that the given SPI block is working as expected, by writing and reading
 * some data via SPI to read the JEDEC/Manufacturer ID of a connected N25Q56A
 * flash memory device, and checking it against known values. This test expects
 * that the SPI pins corresponding to the given SPI block are connected to a
 * device with such memory, e.g. a PMOD SF3.
 *
 * TODO: CS is currently not through pinmux. This is fine to use a GPIO for
 * now, but ideally we would want to actually use CS - when that is changed
 * in the RTL this should be updated to use CS directly instead of taking
 * a GPIO pin to use as CS.
 *
 * Returns true if the test passed, or false if it failed.
 */
static bool spi_n25q256a_read_jedec_id(SpiPtr spi, Capability<volatile SonataGpioFull> gpio, GpioPin spi_cs_pin) {
  constexpr uint8_t CmdReadJEDECId     = 0x9f;
  constexpr uint8_t ExpectedJedecId[3] = {0x20, 0xBA, 0x19};

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Read the JEDEC ID from the external flash
  uint8_t jedec_id[3] = {0x12, 0x34, 0x56};  // (Dummy data)
  set_gpio_output(gpio, spi_cs_pin, false);  // Set ¬CS High
  spi->blocking_write(&CmdReadJEDECId, 1);
  spi->blocking_read(jedec_id, 3);
  set_gpio_output(gpio, spi_cs_pin, true);  // Set ¬CS Low

  // Check that the retrieved ID matches our expected value
  for (size_t index = 0; index < 3; index++) {
    if (jedec_id[index] != ExpectedJedecId[index]) {
      return false;
    }
  }

  return true;
}

/**
 * Execute a UART send/receive test using the UART specified in the test.
 */
static bool execute_uart_test(const Test &test, ds::xoroshiro::P32R8 &prng, UartPtr uarts[4]) {
  UartPtr tested_uart = uarts[static_cast<uint8_t>(test.uart_data.uart) - 1];
  bool result         = uart_send_receive_test(prng, tested_uart, 10, 100);
  return result == test.expected_result;
}

/**
 * Execute a GPIO write/read test using the pins specified in the test.
 */
static bool execute_gpio_test(const Test &test, ds::xoroshiro::P32R8 &prng, Capability<volatile SonataGpioFull> gpio) {
  set_gpio_output_enable(gpio, test.gpio_data.output_pin, true);
  set_gpio_output_enable(gpio, test.gpio_data.input_pin, false);
  bool result = gpio_write_read_test(gpio, test.gpio_data.output_pin, test.gpio_data.input_pin,
                                     test.gpio_data.wait_time, test.gpio_data.test_length);
  return result == test.expected_result;
}

/**
 * Execute an I2C BH1745 ID read test using the I2C specified in the test.
 */
static bool execute_i2c_bh1745_test(const Test &test, I2cPtr i2cs[2]) {
  I2cPtr tested_i2c = i2cs[static_cast<uint8_t>(test.i2c_data.i2c)];
  bool result       = i2c_read_bh1745_id_test(tested_i2c);
  return result == test.expected_result;
}

/**
 * Execute an I2C PMOD Colour ID read test using the I2C specified in the test.
 */
static bool execute_i2c_pmod_colour_test(const Test &test, I2cPtr i2cs[2], Log &log) {
  I2cPtr tested_i2c = i2cs[static_cast<uint8_t>(test.i2c_data.i2c)];
  bool result       = i2c_read_pmod_colour_id_test(tested_i2c, log);
  return result == test.expected_result;
}

/**
 * Execute a SPI N25Q256A ID read test using the SPI and CS pin specified in
 * the test.
 */
static bool execute_spi_test(const Test &test, Capability<volatile SonataGpioFull> gpio, SpiPtr spis[2]) {
  SpiPtr tested_spi = spis[static_cast<uint8_t>(test.spi_data.spi) - 3];
  set_gpio_output_enable(gpio, test.spi_data.cs_pin, true);
  bool result = spi_n25q256a_read_jedec_id(tested_spi, gpio, test.spi_data.cs_pin);
  return result == test.expected_result;
}

/**
 * Checks whether Sonata's joystick is currently pressed down or not.
 */
static inline bool joystick_pressed(Capability<volatile SonataGpioFull> gpio) {
  constexpr uint8_t SonataJoystickPressed = (1 << 2);
  constexpr uint8_t instance              = static_cast<uint8_t>(GpioInstance::General);
  return gpio->instances[instance].input & SonataJoystickPressed;
}

/**
 * Checks whether Sonata's joystick is currently being held in any direction
 * (does not include being pressed down) or not.
 */
static inline bool joystick_moved(Capability<volatile SonataGpioFull> gpio) {
  constexpr uint8_t SonataJoystickMoveMask = 0b11011;
  constexpr uint8_t instance               = static_cast<uint8_t>(GpioInstance::General);
  return (gpio->instances[instance].input & SonataJoystickMoveMask) > 0;
}

/**
 * Execute a given pinmux test as part of an overall test plan. This requires
 * capabilities to all possible devices that might be tested to be provided.
 *
 * @param console Should be UART0, and not in the `uarts` parameter.
 * @param uarts   Should point to UARTS1-4, which are not used as the console.
 * @param spi     Should point to SPI3 and 4, which are not in use by default.
 * @param i2cs    Should point to I2C0 and I2C1.
 *
 * Returns true if the test passed (i.e. got the expected result), and false
 * otherwise.
 */
bool execute_test(const Test &test, uint32_t test_num, Log &log, ds::xoroshiro::P32R8 &prng,
                  Capability<volatile SonataGpioFull> gpio, UartPtr uarts[4], SpiPtr spis[2], I2cPtr i2cs[2],
                  SonataPinmux *pinmux) {
  // If manual intervention is required, print out the test instruction and wait
  // for the user to press the joystick to signal that they are ready.
  if (test.manual_required) {
    log.println("{}{}", prefix, test.instruction);
    log.print("{}\x1b[36mPress the joystick when you are ready to continue.\033[0m", prefix);
    // Wait for the joystick to be released first, so that we don't count any user input from the
    // last test as part of this test.
    while (joystick_pressed(gpio)) {
      asm volatile("");
    }
    while (!joystick_pressed(gpio)) {
      asm volatile("");
    }
    log.println("");
  }
  log.print("{}  Executing test {}: {} ... ", prefix, test_num, test.name);

  // Configure the output pins and block inputs via pinmux
  for (uint32_t i = 0; i < test.num_output_pins; i++) {
    auto pin_assign = test.output_pins[i];
    if (!pinmux->output_pin_select(pin_assign.pin, pin_assign.select)) {
      log.println("");
      log.print("{}Error: invalid pinmux output pin selection.", prefix);
      return false;
    }
  }
  for (uint32_t i = 0; i < test.num_block_inputs; i++) {
    auto block_assign = test.block_inputs[i];
    if (!pinmux->block_input_select(block_assign.input, block_assign.select)) {
      log.println("");
      log.print("{}Error: invalid pinmux block input selection.", prefix);
      return false;
    }
  }

  // Execute the test using the provided test type
  bool passed = false;
  switch (test.type) {
    case TestType::UartSendReceive:
      passed = execute_uart_test(test, prng, uarts);
      break;
    case TestType::GpioWriteRead:
      passed = execute_gpio_test(test, prng, gpio);
      break;
    case TestType::I2cBH1745ReadId:
      passed = execute_i2c_bh1745_test(test, i2cs);
      break;
    case TestType::I2cPmodColourReadId:
      passed = execute_i2c_pmod_colour_test(test, i2cs, log);
      break;
    case TestType::SpiPmodSF3ReadId:
      passed = execute_spi_test(test, gpio, spis);
      break;
    default:
      log.println("");
      log.print("{}Error: unknown test type.", prefix);
  }
  write_test_result(log, (passed ? 0 : 1));
  return passed;
}

/**
 * After a test has failed, this function can be called to ask the user if
 * they would like to retry the previous test, or continue. By pressing the
 * joystick the testplan will continue, but on moving it in any direction
 * the previous test should be repeated.
 *
 * Returns true if the user wishes to retry, and false otherwise.
 */
bool ask_retry_last_test(Log &log, Capability<volatile SonataGpioFull> gpio) {
  log.print("{}\x1b[36mPrevious test failed. Move the joystick to retry, or press it to continue.\033[0m", prefix);

  // Wait for the joystick to be resting first, so that we don't count any accidental user inputs
  // from previous parts of the testing.
  while (joystick_pressed(gpio) || joystick_moved(gpio)) {
    asm volatile("");
  }

  bool was_pressed = false;
  while (!(was_pressed = joystick_pressed(gpio)) && !joystick_moved(gpio)) {
    asm volatile("");
  }
  log.println("");
  if (!was_pressed) {
    log.println("{}\x1b[36mRepeating previous test.\033[0m", prefix);
    return true;
  }
  return false;
}

/**
 * Execute a provided pinmux testplan, which is given as an array of tests that
 * are to be executed sequentially, with users manually following instructions
 * as dictated by the testplan during execution.
 *
 * If `PinmuxCheckFailImmediately` is true, then any single failure will result
 * in the immediate failure of the entire testplan.
 * If instead `PinmuxEnableRetry` is true, then each failure will ask the user
 * whether they wish to retry the last test, and do that if so.
 *
 * @param testplan The testplan (list of tests) to execute.
 * @param NumTests The number of tests to execute from the given testplan.
 *
 * @param console Should be UART0, and not in the `uarts` parameter.
 * @param uarts   Should point to UARTS1-4, which are not used as the console.
 * @param spi     Should point to SPI3 and 4, which are not in use by default.
 * @param i2cs    Should point to I2C0 and I2C1.
 *
 * Returns true if every test passed (i.e. got the expected result), and false
 * otherwise. Will return true even if there were retries.
 */
bool execute_testplan(Test *testplan, uint8_t NumTests, Log &log, ds::xoroshiro::P32R8 &prng,
                      Capability<volatile SonataGpioFull> gpio, UartPtr uarts[4], SpiPtr spis[2], I2cPtr i2cs[2],
                      SonataPinmux *pinmux) {
  log.println("");
  log.println("{}Starting check.", prefix);
  log.println("");

  uint8_t passes        = 0;
  uint8_t retries       = 0;
  uint8_t current_test  = 0;
  bool last_test_passed = true;
  while (current_test < NumTests) {
    // Retry failed tests if enabled and the user wishes to
    if (PinmuxEnableRetry && !last_test_passed && ask_retry_last_test(log, gpio)) {
      current_test--;
      retries++;
    }

    // Execute each test in order.
    Test &test = testplan[current_test];
    current_test++;
    if (!execute_test(test, current_test, log, prng, gpio, uarts, spis, i2cs, pinmux)) {
      last_test_passed = false;
      if (PinmuxCheckFailImmediately)
        break;
      else
        continue;
    }
    last_test_passed = true;
    passes++;
  }

  // Output the combined result of executing the testplan to the console.
  log.println("");
  log.print("{}Check result: {}/{} tests pass", prefix, passes, NumTests);
  if (retries > 0) {
    log.print(" ({} retries)", retries);
  }
  log.print(" ....... ");
  write_test_result(log, static_cast<uint8_t>(passes != current_test));

  return passes == current_test;
}
