/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "pinmux_checker.hh"
#include "../common/uart-utils.hh"

using namespace CHERI;

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))

/**
 * Run a pinmux check testplan which tests the use of the same PMOD pins for
 * GPIO, UART, I2C and SPI transmissions via manual intervention. This
 * provides confidence that the Pinmux is working as expected in the more
 * extreme scenario that all muxed blocks are actually being muxed over the
 * same pins.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Initialise capabilities for UART0 (the console), and all other UARTS (1-4)
  UartPtr uart0 = uart_ptr(rwRoot, 0);
  uart0->init(BAUD_RATE);
  UartPtr uarts[2] = {
      uart_ptr(rwRoot, 1),
      uart_ptr(rwRoot, 2),
  };
  for (UartPtr uart : uarts) {
    uart->init(BAUD_RATE);
  }

  // Create a logging mechanism for UART0.
  WriteUart uart{uart0};
  Log log(uart);

  // Create capabilities for SPI0-2, I2C0&1, GPIO and Pinmux for use in Pinmux testing
  SpiPtr spis[3] = {
      spi_ptr(rwRoot, 2),  // SPI0
      spi_ptr(rwRoot, 3),  // SPI1
      spi_ptr(rwRoot, 4),  // SPI2
  };
  I2cPtr i2cs[2] = {i2c_ptr(root, 0), i2c_ptr(root, 1)};

  SonataGpioFull gpio_full = get_full_gpio_ptrs(root);

  PinmuxPtrs pinmux = std::pair{pin_sinks_ptr(root), block_sinks_ptr(root)};

  // Initialise Pseudo-Random Number Generation for use in Pinmux UART testing
  ds::xoroshiro::P32R8 prng;
  prng.set_state(0xDEAD, 0xBEEF);

  // Define the Pin Output and Block Input Mux settings to be used in the Pinmux
  // testplan. We have to define these separately instead of using nested
  // initializers, as that requires `memcpy` to exist which we do not have.
  // Likewise, arrays with multiple pins/inputs must be individually set, or
  // we will get errors with `memcpy`.
  OutputPinAssignment pmod_test_gpio_on_pins[]     = {{SonataPinmux::PinSink::pmod0_2, 1}};
  BlockInputAssignment pmod_test_gpio_on_inputs[]  = {{SonataPinmux::BlockSink::gpio_2_ios_2, 1}};
  OutputPinAssignment pmod_test_gpio_off_pins[]    = {{SonataPinmux::PinSink::pmod0_2, 0}};
  BlockInputAssignment pmod_test_gpio_off_inputs[] = {{SonataPinmux::BlockSink::gpio_2_ios_2, 0}};

  OutputPinAssignment pmod_test_uart_on_pins[]     = {{SonataPinmux::PinSink::pmod0_2, 4}};
  BlockInputAssignment pmod_test_uart_on_inputs[]  = {{SonataPinmux::BlockSink::uart_1_rx, 5}};
  OutputPinAssignment pmod_test_uart_off_pins[]    = {{SonataPinmux::PinSink::pmod0_2, 0}};
  BlockInputAssignment pmod_test_uart_off_inputs[] = {{SonataPinmux::BlockSink::uart_1_rx, 0}};

  OutputPinAssignment pmod_test_i2c_on_pins[2];
  pmod_test_i2c_on_pins[0] = {SonataPinmux::PinSink::pmod0_3, 2};  // Mux to I2C SDA
  pmod_test_i2c_on_pins[1] = {SonataPinmux::PinSink::pmod0_4, 3};  // Mux to I2C SCL
  OutputPinAssignment pmod_test_i2c_off_pins[2];
  pmod_test_i2c_off_pins[0] = {SonataPinmux::PinSink::pmod0_3, 0};
  pmod_test_i2c_off_pins[1] = {SonataPinmux::PinSink::pmod0_4, 0};

  OutputPinAssignment pmod_test_spi_on_pins[3];
  pmod_test_spi_on_pins[0]                       = {SonataPinmux::PinSink::pmod0_1, 2};  // Mux to SPI CS
  pmod_test_spi_on_pins[1]                       = {SonataPinmux::PinSink::pmod0_2, 2};  // Mux to SPI COPI
  pmod_test_spi_on_pins[2]                       = {SonataPinmux::PinSink::pmod0_4, 2};  // Mux to SPI SCK
  BlockInputAssignment pmod_test_spi_on_inputs[] = {{SonataPinmux::BlockSink::spi_1_cipo, 3}};
  OutputPinAssignment pmod_test_spi_off_pins[3];
  pmod_test_spi_off_pins[0]                       = {SonataPinmux::PinSink::pmod0_1, 0};
  pmod_test_spi_off_pins[1]                       = {SonataPinmux::PinSink::pmod0_2, 0};
  pmod_test_spi_off_pins[2]                       = {SonataPinmux::PinSink::pmod0_4, 0};
  BlockInputAssignment pmod_test_spi_off_inputs[] = {{SonataPinmux::BlockSink::spi_1_cipo, 0}};

  // The pinmux testplan to execute. This testplan runs through testing GPIO, UART, I2C and SPI
  // all on the same PMOD pins, with users manually changing out the connected devices between
  // tests when necessary.
  Test pinmux_testplan[] = {
      {
          .type             = TestType::GpioWriteRead,
          .name             = "PMOD0_2 -> PMOD0_3 GPIO Muxed    ",
          .manual_required  = true,
          .instruction      = "Manually connect PMOD0 Pins 2 & 3 with a wire in a loop.",
          .output_pins      = pmod_test_gpio_on_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_gpio_on_pins),
          .block_inputs     = pmod_test_gpio_on_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_gpio_on_inputs),
          .gpio_data =
              {
                  {GpioInstance::Pmod0, 1},  // PMOD0_2
                  {GpioInstance::Pmod0, 2},  // PMOD0_3
                  GpioWaitUsec,
                  GpioTestLength,
              },
          .expected_result = true,
      },
      {
          .type             = TestType::GpioWriteRead,
          .name             = "PMOD0_2 -> PMOD0_3 GPIO Not Muxed",
          .manual_required  = false,
          .output_pins      = pmod_test_gpio_off_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_gpio_off_pins),
          .block_inputs     = pmod_test_gpio_off_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_gpio_off_inputs),
          .gpio_data =
              {
                  {GpioInstance::Pmod0, 1},  // PMOD0_2
                  {GpioInstance::Pmod0, 2},  // PMOD0_3
                  GpioWaitUsec,
                  GpioTestLength,
              },
          .expected_result = false,
      },
      {
          .type             = TestType::UartSendReceive,
          .name             = "PMOD0_2 -> PMOD0_3 UART Muxed    ",
          .manual_required  = false,
          .output_pins      = pmod_test_uart_on_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_uart_on_pins),
          .block_inputs     = pmod_test_uart_on_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_uart_on_inputs),
          .uart_data =
              {
                  UartTest::UartNum::Uart1,
                  UartTimeoutUsec,
                  UartTestBytes,
              },
          .expected_result = true,
      },
      {
          .type             = TestType::UartSendReceive,
          .name             = "PMOD0_2 -> PMOD0_3 UART Not Muxed",
          .manual_required  = false,
          .output_pins      = pmod_test_uart_off_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_uart_off_pins),
          .block_inputs     = pmod_test_uart_off_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_uart_off_inputs),
          .uart_data =
              {
                  UartTest::UartNum::Uart1,
                  UartTimeoutUsec,
                  UartTestBytes,
              },
          .expected_result = false,
      },
      {
          .type             = TestType::I2cPmodColourReadId,
          .name             = "PMOD0_3 & PMOD0_4 I2C Muxed      ",
          .manual_required  = true,
          .instruction      = "Remove the wire connecting PMOD0 Pins 2 & 3. Connect the PMOD Colour to PMOD0.",
          .output_pins      = pmod_test_i2c_on_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_i2c_on_pins),
          .block_inputs     = nullptr,
          .num_block_inputs = 0,
          .i2c_data         = {I2cTest::I2cNum::I2c0},
          .expected_result  = true,
      },
      {
          .type             = TestType::I2cPmodColourReadId,
          .name             = "PMOD0_3 & PMOD0_4 I2C Not Muxed  ",
          .manual_required  = false,
          .output_pins      = pmod_test_i2c_off_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_i2c_off_pins),
          .block_inputs     = nullptr,
          .num_block_inputs = 0,
          .i2c_data         = {I2cTest::I2cNum::I2c0},
          .expected_result  = false,
      },
      {
          .type             = TestType::SpiPmodSF3ReadId,
          .name             = "PMOD0_{1,2,3,4} SPI Muxed        ",
          .manual_required  = true,
          .instruction      = "Remove the PMOD Colour from PMOD0. Connect the Spi PmodSF3 Flash to PMOD0.",
          .output_pins      = pmod_test_spi_on_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_spi_on_pins),
          .block_inputs     = pmod_test_spi_on_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_spi_on_inputs),
          .spi_data         = {SpiTest::SpiNum::Spi1},
          .expected_result  = true,
      },
      {
          .type             = TestType::SpiPmodSF3ReadId,
          .name             = "PMOD0_{1,2,3,4} SPI Not Muxed    ",
          .manual_required  = false,
          .output_pins      = pmod_test_spi_off_pins,
          .num_output_pins  = ARRAYSIZE(pmod_test_spi_off_pins),
          .block_inputs     = pmod_test_spi_off_inputs,
          .num_block_inputs = ARRAYSIZE(pmod_test_spi_off_inputs),
          .spi_data         = {SpiTest::SpiNum::Spi1},
          .expected_result  = false,
      },
  };

  // Execute the pinmux testplan
  const uint8_t NumTests = ARRAYSIZE(pinmux_testplan);
  execute_testplan(pinmux_testplan, NumTests, log, prng, &gpio_full, uarts, spis, i2cs, pinmux);

  // Infinite loop to stop execution returning
  while (true) {
    asm volatile("");
  }
}
