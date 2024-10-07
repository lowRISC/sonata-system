// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "../../common/defs.h"
#include "../common/console.hh"
#include "../common/flash-utils.hh"
#include "../common/timer-utils.hh"
#include "../common/uart-utils.hh"
#include "../common/sonata-devices.hh"
#include "../common/platform-pinmux.hh"
#include "test_runner.hh"
#include "i2c_tests.hh"
#include <cheri.hh>
#include <platform-uart.hh>
#include <ds/xoroshiro.h>

using namespace CHERI;

/**
 * Configures the number of test iterations to perform.
 * This can be overriden via a compilation flag.
 */
#ifndef PINMUX_TEST_ITERATIONS
#define PINMUX_TEST_ITERATIONS (1U)
#endif

/**
 * Configures whether cable connections required for the pinmux testing
 * are available. This includes cables between:
 *  - mikroBus       (P7) RX & TX
 * This can be overriden via a compilation flag.
 */
#ifndef PINMUX_CABLE_CONNECTIONS_AVAILABLE
#define PINMUX_CABLE_CONNECTIONS_AVAILABLE true
#endif

static constexpr uint8_t PmxToDisabled         = 0;

/**
 * Attempt to retrieve the JEDEC ID of the Spi Flash device on the Sonata board,
 * and compare it to the known (expected) JEDEC ID. This is separate from the
 * similar test in the SPI tests, as it skips the initialisation logic that
 * should not be repeated in the pinmux check.
 *
 * Returns the number of failures during the test.
 */
static int spi_jedec_id_test(Capability<volatile SonataSpi> spi, SpiFlash spi_flash) {
  int failures = 0;

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
 * Test pinmux by enabling and disabling the SPI Flash pins. First reads the
 * Flash's JEDEC ID like normal, then disables the pins via pinmux and repeats
 * the JEDEC ID read, checking that it fails. Then re-enables the pins via
 * pinmux and repeats the JEDEC ID read, checking that it succeeds.
 * Returns the number of failures during the test.
 */
static int pinmux_spi_flash_test(SonataPinmux *pinmux, Capability<volatile SonataSpi> spi, SpiFlash spi_flash) {
  constexpr uint8_t PmxSpiFlashDataToSpiTx0   = 1;
  constexpr uint8_t PmxSpiFlashClockToSpiClk0 = 1;

  int failures = 0;

  // Ensure the SPI Flash pins are enabled using Pinmux
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashData, PmxSpiFlashDataToSpiTx0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashClock, PmxSpiFlashClockToSpiClk0)) failures++;

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Run the normal SPI Flash JEDEC ID Test; it should pass.
  failures += spi_jedec_id_test(spi, spi_flash);

  // Disable the SPI Flash pins through pinmux
  spi->wait_idle();
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashData, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashClock, PmxToDisabled)) failures++;

  // Run the JEDEC ID Test again; we expect it to fail.
  if (spi_jedec_id_test(spi, spi_flash) == 0) failures++;

  // RE-enable the SPI Flash pins through pinmux
  spi->wait_idle();
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashData, PmxSpiFlashDataToSpiTx0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::SpiFlashClock, PmxSpiFlashClockToSpiClk0)) failures++;

  // Run the JEDEC ID Test one more time; it should pass.
  failures += spi_jedec_id_test(spi, spi_flash);

  return failures;
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
 * Test pinmux by enabling and disabling the I2C pins for the Rapberry Pi Sense
 * HAT. This requires the RPi Sense HAT to be connected to the Sonata board, otherwise the
 * test will fail.
 *
 * Runs the I2C Rpi Hat ID EEPROM and WHO_AM_I tests, enabling and disabling the
 * output pins via pinmux and checking that the respective tests fail/succeed
 * as expected according to our Pinmux configuration.
 *
 * Returns the number of failures during the test.
 */
static int pinmux_i2c_test(SonataPinmux *pinmux, I2cPtr i2c0, I2cPtr i2c1) {
  constexpr uint8_t PmxRPiHat27ToI2cSda0 = 1;
  constexpr uint8_t PmxRPiHat28ToI2cScl0 = 1;
  constexpr uint8_t PmxRPiHat3ToI2cSda1  = 1;
  constexpr uint8_t PmxRPiHat5ToI2cScl1  = 1;

  int failures = 0;

  // Ensure the RPI Hat I2C pins are enabled via Pinmux
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat27, PmxRPiHat27ToI2cSda0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat28, PmxRPiHat28ToI2cScl0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat3, PmxRPiHat3ToI2cSda1)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat5, PmxRPiHat5ToI2cScl1)) failures++;

  // Run the normal I2C RPI Hat ID_EEPROM and WHO_AM_I tests
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  // Disable the RPI Hat I2C0 output pins, and check that the ID EEPROM test
  // now fails (and the WHOAMI test still succeeds).
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat27, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat28, PmxToDisabled)) failures++;
  if (i2c_rpi_hat_id_eeprom_test(i2c0) == 0) failures++;
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  // Re-enables the RPI Hat I2C0 pins and disables the I2C1 pins, and check that the
  // ID EEPROM test now passes. and the WHOAMI test now fails.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat27, PmxRPiHat27ToI2cSda0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat28, PmxRPiHat28ToI2cScl0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat3, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat5, PmxToDisabled)) failures++;
  reset_i2c_controller(i2c0);
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  if (i2c_rpi_hat_imu_whoami_test(i2c1) == 0) failures++;

  // Re-enables both the RPI Hat I2C0 and I2C1 pins via pinmux, and checks that both
  // tests now pass again.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat3, PmxRPiHat3ToI2cSda1)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::RaspberryPiHat5, PmxRPiHat5ToI2cScl1)) failures++;
  reset_i2c_controller(i2c1);
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  return failures;
}

/**
 * Run the whole suite of pinmux tests.
 */
void pinmux_tests(CapRoot root, Log &log) {
  // Create a bounded capability for pinmux & initialise the driver
  Capability<volatile uint8_t> pinmux = root.cast<volatile uint8_t>();
  pinmux.address()                    = PINMUX_ADDRESS;
  pinmux.bounds()                     = PINMUX_BOUNDS;
  SonataPinmux Pinmux                 = SonataPinmux(pinmux);

  // Create bounded capabilities for other devices, to be used in testing.
  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS;
  spi.bounds()                       = SPI_BOUNDS;

  SpiFlash spi_flash(spi);

  I2cPtr i2c0 = i2c_ptr(root, 0);
  I2cPtr i2c1 = i2c_ptr(root, 1);

  // Initialise PRNG for use to create (pseudo-)random test data.
  ds::xoroshiro::P32R8 prng;
  prng.set_state(0xDEAD, 0xBEEF);

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < PINMUX_TEST_ITERATIONS; i++) {
    log.println("running pinmux_test: {} \\ {}", i, PINMUX_TEST_ITERATIONS - 1);
    log.println("\x1b[35m(may need manual pin connections to pass)");
    log.println("\x1b[0m\r\n");

    bool test_failed = false;
    int failures     = 0;

    log.print("  Running SPI Flash Pinmux test... ");
    failures = pinmux_spi_flash_test(&Pinmux, spi, spi_flash);
    test_failed |= (failures > 0);
    write_test_result(log, failures);

    if (I2C_RPI_HAT_AVAILABLE) {
      log.print("  Running I2C Pinmux test... ");
      failures = pinmux_i2c_test(&Pinmux, i2c0, i2c1);
      test_failed |= (failures > 0);
      write_test_result(log, failures);
    }

    check_result(log, !test_failed);
  }
}
