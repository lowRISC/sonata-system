/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "../common/sonata-devices.hh"
#include <cheri.hh>
#include <platform-gpio.hh>
#include <ds/xoroshiro.h>
using namespace CHERI;  // The full definition of Sonata's GPIO array.
using SonataGpioFull = std::array<GpioPtr, 6>;

// Enum representing possible GPIO instances, and their ordering
enum class GpioInstance : uint8_t {
  General        = 0,
  RaspberryPiHat = 1,
  ArduinoShield  = 2,
  Pmod0          = 3,
  Pmod1          = 4,
  PmodC          = 5,
};

// A struct for specifying a specific Gpio Pin using its instance & bit
struct GpioPin {
  GpioInstance instance;
  uint8_t bit;  // 0-31 (though it depends on the size of the GPIO instance)
};

SonataGpioFull get_full_gpio_ptrs(Capability<void> CapRoot);
GpioPtr get_gpio_instance(SonataGpioFull *gpio, GpioInstance instance);
void set_gpio_output(SonataGpioFull *gpio, GpioPin pin, bool value);
void set_gpio_output_enable(SonataGpioFull *gpio, GpioPin pin, bool value);
bool get_gpio_input(SonataGpioFull *gpio, GpioPin pin);

bool uart_send_receive_test(ds::xoroshiro::P32R8 &prng, UartPtr uart, uint32_t read_timeout_usec, uint32_t test_length);
bool gpio_write_read_test(SonataGpioFull *gpio, GpioPin output_pin, GpioPin input_pin, uint32_t wait_usec,
                          uint32_t test_length);
bool spi_n25q256a_read_jedec_id(SpiPtr spi);

void reset_i2c_controller(I2cPtr i2c);
