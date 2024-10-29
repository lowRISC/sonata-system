/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "block_tests.hh"
#include "../common/timer-utils.hh"

// CPU clock cycles per microsecond, rounding up because this is going to be a fairly small
// integral number and the clock frequency may not be an integral number of MHz.
static constexpr uint32_t CyclesPerMicrosecond = (CPU_TIMER_HZ + 999'999) / 1'000'000;

/**
 * Sets the value of some GPIO output pin. This change will only be visible
 * if the pin has its corresponding `output_enable` bit set to 1, so that the
 * pin is in output mode.
 */
void set_gpio_output(SonataGpioFull *gpio, GpioPin pin, bool value) {
  switch (pin.instance) {
    case GpioInstance::General:
      return gpio->general->set_output(pin.bit, value);
    case GpioInstance::RaspberryPiHat:
      return gpio->rpi->set_output(pin.bit, value);
    case GpioInstance::ArduinoShield:
      return gpio->arduino->set_output(pin.bit, value);
    case GpioInstance::Pmod0:
      return gpio->pmod0->set_output(pin.bit, value);
    case GpioInstance::Pmod1:
      return gpio->pmod1->set_output(pin.bit, value);
  }
}

/**
 * Sets the output enable value of some GPIO pin. Setting this to 1 places the
 * pin in output mode, and to 0 places it in input mode.
 */
void set_gpio_output_enable(SonataGpioFull *gpio, GpioPin pin, bool value) {
  switch (pin.instance) {
    case GpioInstance::General:
      return gpio->general->set_output_enable(pin.bit, value);
    case GpioInstance::RaspberryPiHat:
      return gpio->rpi->set_output_enable(pin.bit, value);
    case GpioInstance::ArduinoShield:
      return gpio->arduino->set_output_enable(pin.bit, value);
    case GpioInstance::Pmod0:
      return gpio->pmod0->set_output_enable(pin.bit, value);
    case GpioInstance::Pmod1:
      return gpio->pmod1->set_output_enable(pin.bit, value);
  }
}

/**
 * Get the value of some GPIO input pin. This will only be visible if the pin
 * has its corresponding `output_enable` bit set to 0, so that the pin is in
 * input mode.
 */
bool get_gpio_input(SonataGpioFull *gpio, GpioPin pin) {
  switch (pin.instance) {
    case GpioInstance::General:
      return gpio->general->read_input(pin.bit);
    case GpioInstance::RaspberryPiHat:
      return gpio->rpi->read_input(pin.bit);
    case GpioInstance::ArduinoShield:
      return gpio->arduino->read_input(pin.bit);
    case GpioInstance::Pmod0:
      return gpio->pmod0->read_input(pin.bit);
    case GpioInstance::Pmod1:
      return gpio->pmod1->read_input(pin.bit);
  }
}

/**
 * Tests that the given UART appears to be working as expected, by sending and
 * receiving some data using the given UART block. To pass, this test expects
 * the Tx and the Rx of the specified UART to be manually connected on the
 * board, so that data that is written can be read back by the board.
 *
 * @param read_timeout_usec The timeout in microseconds provided to read each
 * individual byte sent over UART. Any timeout causes immediate failure.
 * @param test_length The number of random bytes to try sending over UART.
 *
 * Returns true if the test passed, or false if it failed.
 */
bool uart_send_receive_test(ds::xoroshiro::P32R8 &prng, UartPtr uart, uint32_t read_timeout_usec,
                            uint32_t test_length) {
  constexpr uint8_t UartStatusRegTxIdle = (1 << 2);
  constexpr uint8_t UartStatusRegRxIdle = (1 << 4);

  // Re-initialise the UART and clear its FIFOs.
  uart->init();
  uart->fifos_clear();
  uart->parity();
  reset_mcycle();

  // Wait with timeout for the UART TX FIFO to be empty and all bits to be transmitted,
  // and the RX to be idle; if the Rx input is low continually then the UART block may remain
  // active attempting to receive apparent traffic.
  const uint32_t start_mcycle   = get_mcycle();
  const uint32_t cycles         = read_timeout_usec * CyclesPerMicrosecond;
  const uint32_t timeout_mcycle = start_mcycle + cycles;
  while ((uart->status & UartStatusRegTxIdle) == 0 || (uart->status & UartStatusRegRxIdle) == 0) {
    // Has the timeout occurred yet?
    if (timeout_mcycle < get_mcycle()) {
      return false;
    }
  }
  uart->fifos_clear();

  // Send and receive a test message, character by character
  for (uint32_t i = 0; i < test_length; ++i) {
    const uint8_t test_byte = prng();
    uart->blocking_write(test_byte);

    // Try to read the character back within the given timeout.
    const uint32_t start_mcycle   = get_mcycle();
    const uint32_t cycles         = read_timeout_usec * CyclesPerMicrosecond;
    const uint32_t timeout_mcycle = start_mcycle + cycles;
    while (!uart->can_read() && get_mcycle() < timeout_mcycle) {
      asm("");
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
 * @param wait_usec The time in microseconds to wait for the GPIO write signal
 * to be transmitted before reading.
 * @param test_length The number of individual write/reads to test on the given
 * GPIO connection.
 *
 * Returns true if the test passed, or false if it failed.
 */
bool gpio_write_read_test(SonataGpioFull *gpio, GpioPin output_pin, GpioPin input_pin, uint32_t wait_usec,
                          uint32_t test_length) {
  bool gpio_high = false;
  for (uint32_t i = 0; i < test_length; i++) {
    // For each test iteration, invert the GPIO signal
    gpio_high = !gpio_high;

    // Write this inverted signal on the configured output GPIO
    set_gpio_output(gpio, output_pin, gpio_high);

    // Wait a short time for the transmission
    const uint32_t cycles     = wait_usec * CyclesPerMicrosecond;
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
void reset_i2c_controller(I2cPtr i2c) {
  i2c->control = i2c->control & ~(i2c->ControlEnableHost | i2c->ControlEnableTarget);
  if (i2c->interrupt_is_asserted(OpenTitanI2cInterrupt::ControllerHalt)) {
    i2c->reset_controller_events();
  }
}
