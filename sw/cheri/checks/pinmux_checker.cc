/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pinmux_checker.hh"
#include "../common/timer-utils.hh"

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
 * Returns true if the test passed, or false if it failed.
 */
static bool spi_n25q256a_read_jedec_id(SpiPtr spi) {
  constexpr uint8_t CmdReadJEDECId     = 0x9f;
  constexpr uint8_t ExpectedJedecId[3] = {0x20, 0xBA, 0x19};

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Read the JEDEC ID from the external flash
  uint8_t jedec_id[3] = {0x12, 0x34, 0x56};  // (Dummy data)
  spi->cs             = (spi->cs & ~1u);     // Set ¬CS High
  spi->blocking_write(&CmdReadJEDECId, 1);
  spi->blocking_read(jedec_id, 3);
  spi->cs = (spi->cs | 1u);  // Set ¬CS Low

  // Check that the retrieved ID matches our expected value
  for (size_t index = 0; index < sizeof(jedec_id); index++) {
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
  UartPtr tested_uart = uarts[static_cast<uint8_t>(test.uart_data.uart)];
  bool result         = uart_send_receive_test(prng, tested_uart, test.uart_data.timeout, test.uart_data.test_length);
  return result == test.expected_result;
}

/**
 * Execute a GPIO write/read test using the pins specified in the test.
 */
static bool execute_gpio_test(const Test &test, ds::xoroshiro::P32R8 &prng, SonataGpioFull *gpio) {
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
static bool execute_spi_test(const Test &test, SpiPtr spis[2]) {
  SpiPtr tested_spi = spis[static_cast<uint8_t>(test.spi_data.spi)];
  bool result       = spi_n25q256a_read_jedec_id(tested_spi);
  return result == test.expected_result;
}

/**
 * Checks whether Sonata's joystick is currently pressed down or not.
 */
static inline bool joystick_pressed(SonataGpioFull *gpio) {
  constexpr uint8_t SonataJoystickPressed = (1 << 2);
  return gpio->general->input & SonataJoystickPressed;
}

/**
 * Checks whether Sonata's joystick is currently being held in any direction
 * (does not include being pressed down) or not.
 */
static inline bool joystick_moved(SonataGpioFull *gpio) {
  constexpr uint8_t SonataJoystickMoveMask = 0b11011;
  return (gpio->general->input & SonataJoystickMoveMask) > 0;
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
bool execute_test(const Test &test, uint32_t test_num, Log &log, ds::xoroshiro::P32R8 &prng, SonataGpioFull *gpio,
                  UartPtr uarts[4], SpiPtr spis[2], I2cPtr i2cs[2], SonataPinmux *pinmux) {
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
      passed = execute_spi_test(test, spis);
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
bool ask_retry_last_test(Log &log, SonataGpioFull *gpio) {
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
bool execute_testplan(Test *testplan, uint8_t NumTests, Log &log, ds::xoroshiro::P32R8 &prng, SonataGpioFull *gpio,
                      UartPtr uarts[4], SpiPtr spis[2], I2cPtr i2cs[2], SonataPinmux *pinmux) {
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
