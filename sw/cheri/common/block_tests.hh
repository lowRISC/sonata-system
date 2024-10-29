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

using namespace CHERI;

// The full definition of Sonata's GPIO array.
struct SonataGpioFull {
  Capability<volatile SonataGpioGeneral> general;
  Capability<volatile SonataGpioRaspberryPiHat> rpi;
  Capability<volatile SonataGpioArduinoShield> arduino;
  Capability<volatile SonataGpioPmod> pmod0;
  Capability<volatile SonataGpioPmod> pmod1;
};

// Enum representing possible GPIO instances, and their ordering
enum class GpioInstance : uint8_t {
  General        = 0,
  RaspberryPiHat = 1,
  ArduinoShield  = 2,
  Pmod0          = 3,
  Pmod1          = 4,
};

// A struct for specifying a specific Gpio Pin using its instance & bit
struct GpioPin {
  GpioInstance instance;
  uint8_t bit;  // 0-31 (though it depends on the size of the GPIO instance)
};

void set_gpio_output(SonataGpioFull *gpio, GpioPin pin, bool value);
void set_gpio_output_enable(SonataGpioFull *gpio, GpioPin pin, bool value);
bool get_gpio_input(SonataGpioFull *gpio, GpioPin pin);

bool uart_send_receive_test(ds::xoroshiro::P32R8 &prng, UartPtr uart, uint32_t read_timeout_usec, uint32_t test_length);
bool gpio_write_read_test(SonataGpioFull *gpio, GpioPin output_pin, GpioPin input_pin, uint32_t wait_usec,
                          uint32_t test_length);

void reset_i2c_controller(I2cPtr i2c);
