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
 * Retrieve a struct containing capabilities to all of Sonata's individual GPIO
 * devices / MMIO regions. These GPIO capabilities are all bounded appropriately
 * and kept in separate, isolated capabilities.
 */
SonataGpioFull get_full_gpio_ptrs(Capability<void> root) {
  SonataGpioFull gpio_full;
  for (uint8_t idx = 0; idx < gpio_full.size(); ++idx) {
    gpio_full[idx] = gpio_ptr(root, idx);
  }
  return gpio_full;
}

/**
 * Get a capability for a specific GPIO instance from the full range of GPIO.
 */
GpioPtr get_gpio_instance(SonataGpioFull *gpio, GpioInstance instance) {
  return (*gpio)[static_cast<size_t>(instance)];
}

/**
 * Sets the value of some GPIO output pin. This change will only be visible
 * if the pin has its corresponding `output_enable` bit set to 1, so that the
 * pin is in output mode.
 */
void set_gpio_output(SonataGpioFull *gpio, GpioPin pin, bool value) {
  get_gpio_instance(gpio, pin.instance)->set_output(pin.bit, value);
}

/**
 * Sets the output enable value of some GPIO pin. Setting this to 1 places the
 * pin in output mode, and to 0 places it in input mode.
 */
void set_gpio_output_enable(SonataGpioFull *gpio, GpioPin pin, bool value) {
  get_gpio_instance(gpio, pin.instance)->set_output_enable(pin.bit, value);
}

/**
 * Get the value of some GPIO input pin. This will only be visible if the pin
 * has its corresponding `output_enable` bit set to 0, so that the pin is in
 * input mode.
 */
bool get_gpio_input(SonataGpioFull *gpio, GpioPin pin) {
  return get_gpio_instance(gpio, pin.instance)->read_input(pin.bit);
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
 * Tests that the given SPI block is working as expected, by writing and reading
 * some data via SPI to read the JEDEC/Manufacturer ID of a connected N25Q56A
 * flash memory device, and checking it against known values. This test expects
 * that the SPI pins corresponding to the given SPI block are connected to a
 * device with such memory, e.g. a PMOD SF3.
 *
 * Returns true if the test passed, or false if it failed.
 */
bool spi_n25q256a_read_jedec_id(SpiPtr spi) {
  constexpr uint8_t CmdReadJEDECId     = 0x9f;
  constexpr uint8_t ExpectedJedecId[3] = {0x20, 0xBA, 0x19};

  // Configure the SPI to be MSB-first.
  spi->wait_idle();
  spi->init(false, false, true, 0);

  // Read the JEDEC ID from the external flash
  uint8_t jedec_id[3] = {0x12, 0x34, 0x56};  // (Dummy data)
  spi->chip_select_assert<0>();
  spi->blocking_write(&CmdReadJEDECId, 1);
  spi->blocking_read(jedec_id, 3);
  spi->chip_select_assert<0>(false);

  // Check that the retrieved ID matches our expected value
  for (size_t index = 0; index < sizeof(jedec_id); index++) {
    if (jedec_id[index] != ExpectedJedecId[index]) {
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
  if (i2c->interrupt_is_asserted(OpenTitanI2c::Interrupt::ControllerHalt)) {
    i2c->reset_controller_events();
  }
}
