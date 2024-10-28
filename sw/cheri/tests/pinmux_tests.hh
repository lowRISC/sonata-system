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
#include "../common/block_tests.hh"
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
 *  - Arduino Shield (P4) D0 & D1
 *  - Arduino Shield (P4) D8 & D9
 * This can be overriden via a compilation flag.
 */
#ifndef PINMUX_CABLE_CONNECTIONS_AVAILABLE
#define PINMUX_CABLE_CONNECTIONS_AVAILABLE true
#endif

// Testing parameters
static constexpr uint32_t UartTimeoutUsec = 24;  // with 921,600 bps, this is > 25 bit times.
static constexpr uint32_t UartTestBytes   = 100;
static constexpr uint32_t GpioWaitUsec    = 20;  // short wire bridge between FGPA pins.
static constexpr uint32_t GpioTestLength  = 10;

static constexpr uint8_t PmxToDisabled = 0;

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
 * Test pinmux by enabling and disabling the UART3 TX pin output and UART3 RX
 * block input. Tests the UART itself by sending and receiving some data over
 * UART3; it is required that UART3 TX and RX are manually connected on Sonata
 * for this test (mikroBus P7 RX & TX).
 *
 * Tests UART3 normally, then disables TX and checks the test fails, then
 * disables RX and checks the test fails, and then re-enables the pins and
 * repeats the test, checking that it succeeds.
 *
 * Returns the number of failures during the test.
 */
static int pinmux_uart_test(SonataPinmux *pinmux, ds::xoroshiro::P32R8 &prng, UartPtr uart3) {
  constexpr uint8_t PmxMikroBusUartTransmitToUartTx3 = 1;
  constexpr uint8_t PmxUartReceive3ToMb8             = 2;

  int failures = 0;

  // Mux UART3 over mikroBus P7 RX & TX via default.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::mb7, PmxMikroBusUartTransmitToUartTx3)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::uart_3_rx, PmxUartReceive3ToMb8)) failures++;

  // Check that messages are sent and received via UART3
  if (!uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;

  // Disable UART3 TX through pinmux, and check the test now fails (no TX sent)
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::mb7, PmxToDisabled)) failures++;
  if (uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;

  // Re-enable UART3 TX and disable UART3 RX through pinmux, and check that the test
  // still fails (no RX received)
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::mb7, PmxMikroBusUartTransmitToUartTx3)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::uart_3_rx, PmxToDisabled)) failures++;
  if (uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;

  // Re-enable UART3 RX and check the test now passes again
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::uart_3_rx, PmxUartReceive3ToMb8)) failures++;
  if (!uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;

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
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_d0, PmxSpiFlashDataToSpiTx0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_clk, PmxSpiFlashClockToSpiClk0)) failures++;

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Run the normal SPI Flash JEDEC ID Test; it should pass.
  failures += spi_jedec_id_test(spi, spi_flash);

  // Disable the SPI Flash pins through pinmux
  spi->wait_idle();
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_d0, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_clk, PmxToDisabled)) failures++;

  // Run the JEDEC ID Test again; we expect it to fail.
  if (spi_jedec_id_test(spi, spi_flash) == 0) failures++;

  // RE-enable the SPI Flash pins through pinmux
  spi->wait_idle();
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_d0, PmxSpiFlashDataToSpiTx0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::appspi_clk, PmxSpiFlashClockToSpiClk0)) failures++;

  // Run the JEDEC ID Test one more time; it should pass.
  failures += spi_jedec_id_test(spi, spi_flash);

  return failures;
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
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g0, PmxRPiHat27ToI2cSda0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g1, PmxRPiHat28ToI2cScl0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g2_sda, PmxRPiHat3ToI2cSda1)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g3_scl, PmxRPiHat5ToI2cScl1)) failures++;

  // Run the normal I2C RPI Hat ID_EEPROM and WHO_AM_I tests
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  // Disable the RPI Hat I2C0 output pins, and check that the ID EEPROM test
  // now fails (and the WHOAMI test still succeeds).
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g0, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g1, PmxToDisabled)) failures++;
  if (i2c_rpi_hat_id_eeprom_test(i2c0) == 0) failures++;
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  // Re-enables the RPI Hat I2C0 pins and disables the I2C1 pins, and check that the
  // ID EEPROM test now passes. and the WHOAMI test now fails.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g0, PmxRPiHat27ToI2cSda0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g1, PmxRPiHat28ToI2cScl0)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g2_sda, PmxToDisabled)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g3_scl, PmxToDisabled)) failures++;
  reset_i2c_controller(i2c0);
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  if (i2c_rpi_hat_imu_whoami_test(i2c1) == 0) failures++;

  // Re-enables both the RPI Hat I2C0 and I2C1 pins via pinmux, and checks that both
  // tests now pass again.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g2_sda, PmxRPiHat3ToI2cSda1)) failures++;
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::rph_g3_scl, PmxRPiHat5ToI2cScl1)) failures++;
  reset_i2c_controller(i2c1);
  failures += i2c_rpi_hat_id_eeprom_test(i2c0);
  failures += i2c_rpi_hat_imu_whoami_test(i2c1);

  return failures;
}

/**
 * Test pinmux by enabling and disabling GPIO pins on the Arduino Shield header. This
 * requires that there is a cable connecting pins D8 and D9 on the Arduino Shield
 * header of the Sonata board. This tests writing to and reading from GPIO pins,
 * checking that the respective tests fail/succeed as expected according to our
 * pinmux configuration, enabling and disabling these GPIO via pinmux.
 *
 * Returns the number of failures durign the test.
 */
static int pinmux_gpio_test(SonataPinmux *pinmux, SonataGpioFull *gpio) {
  constexpr uint8_t PmxArduinoD8ToGpios_1_8   = 1;
  constexpr uint8_t PmxArduinoGpio9ToAhTmpio9 = 1;

  constexpr GpioPin GpioPinInput  = {GpioInstance::ArduinoShield, 9};
  constexpr GpioPin GpioPinOutput = {GpioInstance::ArduinoShield, 8};

  int failures = 0;

  // Configure the Arduino D9 GPIO as input and D8 as output.
  set_gpio_output_enable(gpio, GpioPinOutput, true);
  set_gpio_output_enable(gpio, GpioPinInput, false);

  // Ensure the GPIO (Arduino Shield D8 & D9) are enabled via Pinmux
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio8, PmxArduinoD8ToGpios_1_8)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::gpio_1_ios_9, PmxArduinoGpio9ToAhTmpio9)) failures++;

  // Check that reading & writing from/to GPIO works as expected.
  if (!gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

  // Disable the GPIO via pinmux, and check that the test now fails.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio8, PmxToDisabled)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::gpio_1_ios_9, PmxToDisabled)) failures++;
  if (gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

  // Re-enable the GPIO via pinmux, and check that the test passes once more
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio8, PmxArduinoD8ToGpios_1_8)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::gpio_1_ios_9, PmxArduinoGpio9ToAhTmpio9)) failures++;
  if (!gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

  return failures;
}

/**
 * Test the muxing capability of pinmux, by dynamically switching between using
 * (and testing) UART and pinmux on the same two pins - specifically the Arduino
 * Shield header D0 and D1. It is required that these two pins are manually
 * connected on the Sonata board for this test.
 *
 * This tests that when muxed to UART, the corresponding UART send/receive test
 * works (and GPIO does not), and when muxed to GPIO, the corresponding GPIO
 * write/read test works (and UART does not).
 *
 * Returns the number of failures during the test.
 */
static int pinmux_mux_test(SonataPinmux *pinmux, ds::xoroshiro::P32R8 &prng, UartPtr uart3, SonataGpioFull *gpio) {
  constexpr uint8_t PmxArduinoD1ToUartTx3     = 2;
  constexpr uint8_t PmxArduinoD1ToGpio_1_1    = 1;
  constexpr uint8_t PmxUartReceive3ToAhTmpio0 = 1;
  constexpr uint8_t PmxArduinoGpio0ToAhTmpio0 = 1;

  constexpr GpioPin GpioPinInput  = {GpioInstance::ArduinoShield, 0};
  constexpr GpioPin GpioPinOutput = {GpioInstance::ArduinoShield, 1};

  int failures = 0;

  // Set the Arduino GPIO D0 as input and D1 as output.
  set_gpio_output_enable(gpio, GpioPinOutput, true);
  set_gpio_output_enable(gpio, GpioPinInput, false);

  // Mux UART3 over Arduino Shield D0 (RX) & D1 (TX)
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio1, PmxArduinoD1ToUartTx3)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::uart_3_rx, PmxUartReceive3ToAhTmpio0)) failures++;

  // Test that UART3 works over the muxed Arduino Shield D0 & D1 pins,
  // and that GPIO does not work, as these pins are not muxed for GPIO.
  if (!uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;
  if (gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

  // Mux GPIO over Arduino Shield D0 (GPIO input) & D1 (GPIO output)
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio1, PmxArduinoD1ToGpio_1_1)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::gpio_1_ios_0, PmxArduinoGpio0ToAhTmpio0)) failures++;

  // Test that UART3 no longer works (no longer muxed over D0 & D1),
  // and that our muxed GPIO now works.
  if (uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;
  if (!gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

  // Mux back to UART3 again, and test that UART again passes and GPIO fails.
  if (!pinmux->output_pin_select(SonataPinmux::OutputPin::ah_tmpio1, PmxArduinoD1ToUartTx3)) failures++;
  if (!pinmux->block_input_select(SonataPinmux::BlockInput::uart_3_rx, PmxUartReceive3ToAhTmpio0)) failures++;
  if (!uart_send_receive_test(prng, uart3, UartTimeoutUsec, UartTestBytes)) failures++;
  if (gpio_write_read_test(gpio, GpioPinOutput, GpioPinInput, GpioWaitUsec, GpioTestLength)) failures++;

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

  // Create bounded capabilities for the full range of GPIO
  SonataGpioFull gpio_full;
  gpio_full.general           = root.cast<volatile SonataGpioGeneral>();
  gpio_full.general.address() = GPIO_ADDRESS;
  gpio_full.general.bounds()  = GPIO_BOUNDS;
  gpio_full.rpi               = root.cast<volatile SonataGpioRaspberryPiHat>();
  gpio_full.rpi.address()     = GPIO_ADDRESS + GPIO_RANGE;
  gpio_full.rpi.bounds()      = GPIO_BOUNDS;
  gpio_full.arduino           = root.cast<volatile SonataGpioArduinoShield>();
  gpio_full.arduino.address() = GPIO_ADDRESS + GPIO_RANGE * 2;
  gpio_full.arduino.bounds()  = GPIO_BOUNDS;
  gpio_full.pmod              = root.cast<volatile SonataGpioPmod>();
  gpio_full.pmod.address()    = GPIO_ADDRESS + GPIO_RANGE * 3;
  gpio_full.pmod.bounds()     = GPIO_BOUNDS;

  SpiFlash spi_flash(spi);

  UartPtr uart3 = uart_ptr(root, 3);

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

    if (PINMUX_CABLE_CONNECTIONS_AVAILABLE) {
      log.print("  Running GPIO Pinmux test... ");
      failures = pinmux_gpio_test(&Pinmux, &gpio_full);
      test_failed |= (failures > 0);
      write_test_result(log, failures);

      log.print("  Running UART Pinmux test... ");
      failures = pinmux_uart_test(&Pinmux, prng, uart3);
      test_failed |= (failures > 0);
      write_test_result(log, failures);

      log.print("  Running Mux test... ");
      failures = pinmux_mux_test(&Pinmux, prng, uart3, &gpio_full);
      test_failed |= (failures > 0);
      write_test_result(log, failures);
    }

    check_result(log, !test_failed);
  }
}
