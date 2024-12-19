// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "../../common/defs.h"
#include "../common/console.hh"
#include "../common/block_tests.hh"
#include "../common/timer-utils.hh"
#include "test_runner.hh"
#include <cheri.hh>
#include <platform-gpio.hh>

using namespace CHERI;

/**
 * Configures the number of test iterations to perform.
 * This can be overridden via a compilation flag.
 */
#ifndef PWM_TEST_ITERATIONS
#define PWM_TEST_ITERATIONS (1U)
#endif

/**
 * Configures whether cable connections required for PWM testing
 * are available. This includes cables between:
 *  - MikroBus PWM and PMOD0 Pin 1
 * This can be overridden via a compilation flag.
 */
#ifndef PWM_CABLE_CONNECTIONS_AVAILABLE
#define PWM_CABLE_CONNECTIONS_AVAILABLE true
#endif

// Test Parameters
constexpr uint32_t NumPwmCyclesObserved  = 25;
constexpr uint32_t AllowedCycleDeviation = 20;
constexpr bool LogPwmTests               = false;

/**
 * Runs a PWM loopback tests, which uses a jumper cable connecting a PWM output pin to a GPIO input pin to perform
 * measurements of the PWM output, and test whether the PWM is acting correctly as configured within a certain margin of
 * error.
 *
 * Requires a connection between PWM 0 (mikroBus PWM) and PMOD0 pin 1.
 *
 * @param gpio_pmod0 Capability for PMOD0's GPIO
 * @param pwm PWM driver
 * @param period The PWM period to test with
 * @param duty_cycle The PWM duty cycle to test with. Represented as a separate counter, so for example a duty cycle of
 * 50 with a period of 200 would give 1/4 i.e. 25% high output.
 * @param cycles The number of PWM cycles to test
 * @param error_delta The margin of error on either side of the number of cycles measured for the period and duty cycle.
 * There will always be some error, as measuring logic cannot run on every cycle.
 *
 * @returns The integer number of failures during the test
 */
int pwm_loopback_test(Capability<volatile SonataGpioPmod0> gpio_pmod0, uint8_t input_pin, PwmPtr pwm,
                      uint8_t pwm_instance, uint8_t period, uint8_t duty_cycle, uint8_t cycles, uint8_t error_delta,
                      Log &log) {
  // Calculate error bounds & timeouts for use in testing
  const uint32_t period_lower     = period - (error_delta > period ? period : error_delta);
  const uint32_t period_upper     = period + error_delta;
  const uint32_t duty_cycle_lower = duty_cycle - (error_delta > duty_cycle ? duty_cycle : error_delta);
  const uint32_t duty_cycle_upper = duty_cycle + error_delta;
  const uint32_t timeout          = period * 2;

  int failures = 0;

  // Configure GPIO, timer and PWM respectively.
  gpio_pmod0->set_output_enable(input_pin, false);
  reset_mcycle();
  pwm->output_set(pwm_instance, period, duty_cycle);

  uint32_t cycles_observed = 0;
  const bool inputState    = gpio_pmod0->read_input(input_pin);
  while (cycles_observed < cycles) {
    // Wait for the next cycle to avoid missing measurements due to arithmetic/logging.
    uint32_t start_mcycle  = get_mcycle();
    uint32_t timeout_cycle = start_mcycle + timeout;
    while (gpio_pmod0->read_input(input_pin) == inputState && get_mcycle() < timeout_cycle) {
      asm volatile("");
    }
    start_mcycle  = get_mcycle();
    timeout_cycle = start_mcycle + timeout;
    while (gpio_pmod0->read_input(input_pin) != inputState && get_mcycle() < timeout_cycle) {
      asm volatile("");
    }

    // Measure PWM period & duty cycle times
    start_mcycle  = get_mcycle();
    timeout_cycle = start_mcycle + timeout;
    while (gpio_pmod0->read_input(input_pin) == inputState && get_mcycle() < timeout_cycle) {
      asm volatile("");
    }
    uint32_t changed_mcycle = get_mcycle();
    timeout_cycle           = changed_mcycle + timeout;
    while (gpio_pmod0->read_input(input_pin) != inputState && get_mcycle() < timeout_cycle) {
      asm volatile("");
    }
    uint32_t end_mcycle = get_mcycle();

    // Check that our measured cycles are within expected deviation
    const uint32_t period_cycles   = end_mcycle - start_mcycle;
    const uint32_t first_cycles    = changed_mcycle - start_mcycle;
    const uint32_t second_cycles   = end_mcycle - changed_mcycle;
    const uint32_t measured_cycles = inputState ? first_cycles : second_cycles;
    failures += (period_cycles < period_lower);
    failures += (period_cycles > period_upper);
    failures += (measured_cycles < duty_cycle_lower);
    failures += (measured_cycles > duty_cycle_upper);
    cycles_observed++;

    // Optionally log output for debugging purposes
    if (LogPwmTests) {
      log.println("PWM cycle {}:", cycles_observed);
      log.println("Start: {}, Changed: {}, End: {}", start_mcycle, changed_mcycle, end_mcycle);
      log.println("Period test: {} < {} < {}", period_lower, period_cycles, period_upper);
      log.println("Duty cycle test: {} < {} < {}", duty_cycle_lower, measured_cycles, duty_cycle_upper);
    }
  }

  return failures;
}

/**
 * Run the PWM "Zero Counter" test. This checks that when the PWM counter i.e. period is set to 0, that no matter the
 * configured duty cycle, the PWM will only ever output 0.
 *
 * Requires a connection between PWM 0 (mikroBus PWM) and PMOD0 pin 1.
 *
 * @returns The integer number of failures during the test.
 */
int pwm_zero_counter_test(Capability<volatile SonataGpioPmod0> gpio_pmod0, Capability<volatile SonataPwm> pwm) {
  constexpr uint8_t PwmInstance        = 0;
  constexpr uint8_t InputPin           = 0;
  constexpr uint8_t PropagationWaitOps = 25;

  int failures = 0;

  // Configure GPIO, timer and PWM respectively.
  gpio_pmod0->set_output_enable(InputPin, false);
  reset_mcycle();
  pwm->output_set(PwmInstance, 0, 0);

  for (uint32_t duty_cycle = 0; duty_cycle <= 255; duty_cycle++) {
    pwm->output_set(PwmInstance, 0, duty_cycle);
    // Wait a (very) short while to be sure the signal propagated
    for (uint32_t i = 0; i < PropagationWaitOps; i++) {
      asm volatile("nop");
    }
    failures += gpio_pmod0->read_input(0);
  }

  return failures;
}

/**
 * Run the PWM "Always High" test. This checks that when the PWM duty cycle exceeds its counter (i.e. its period),
 * that the PWM will only ever output 1.
 *
 * Requires a connection between PWM 0 (mikroBus PWM) and PMOD0 pin 1.
 *
 * @returns The integer number of failures during the test.
 */
int pwm_always_high_test(Capability<volatile SonataGpioPmod0> gpio_pmod0, Capability<volatile SonataPwm> pwm) {
  constexpr uint8_t PwmInstance = 0;
  constexpr uint8_t InputPin    = 0;

  int failures = 0;

  // Configure GPIO, timer and PWM respectively.
  gpio_pmod0->set_output_enable(InputPin, false);
  reset_mcycle();
  pwm->output_set(PwmInstance, 0, 1);

  for (uint32_t period = 1; period < 255; period++) {
    // Choose a duty cycle about 50% between the period end and the max (255)
    uint32_t duty_cycle = (256 - period) / 2 + period;
    pwm->output_set(PwmInstance, period, duty_cycle);
    // Check that output is always high for 2 PWM periods (+ a small constant)
    constexpr uint8_t MinimumCycles = 50;
    uint32_t start_mcycle           = get_mcycle();
    uint32_t end_mcycle             = start_mcycle + (period * 2) + MinimumCycles;
    while (get_mcycle() < end_mcycle) {
      failures += !gpio_pmod0->read_input(0);
    }
  }

  // Disable PWM output
  pwm->output_set(PwmInstance, 0, 0);

  return failures;
}

/**
 * Run the whole suite of PWM tests.
 */
void pwm_tests(CapRoot root, Log &log) {
  // Create bounded capabilities for PMOD0's GPIO
  Capability<volatile SonataGpioPmod0> gpio_pmod0 = root.cast<volatile SonataGpioPmod0>();
  gpio_pmod0.address()                            = GPIO_ADDRESS + GPIO_RANGE * 3;
  gpio_pmod0.bounds()                             = GPIO_BOUNDS;

  // Create bounded capability for PWM
  PwmPtr pwm = pwm_ptr(root);

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < PWM_TEST_ITERATIONS; i++) {
    log.println("\r\nrunning pwm_test: {} \\ {}", i, PWM_TEST_ITERATIONS - 1);
    set_console_mode(log, CC_PURPLE);
    log.println("(needs manual pin connections to pass)");
    set_console_mode(log, CC_RESET);
    bool test_failed = false;
    int failures     = 0;

    if (PWM_CABLE_CONNECTIONS_AVAILABLE) {
      constexpr size_t NumTests               = 8;
      constexpr uint8_t periods[NumTests]     = {255, 255, 255, 255, 233, 128, 128, 64};
      constexpr uint8_t duty_cycles[NumTests] = {224, 192, 128, 64, 74, 64, 32, 32};

      for (size_t i = 0; i < NumTests; i++) {
        uint8_t period     = periods[i];
        uint8_t duty_cycle = duty_cycles[i];
        log.print("  Running PWM Loopback ({}/{}) test... ", duty_cycle, period);
        failures = pwm_loopback_test(gpio_pmod0, 0, pwm, 0, period, duty_cycle, NumPwmCyclesObserved,
                                     AllowedCycleDeviation, log);
        test_failed |= (failures > 0);
        write_test_result(log, failures);
      }

      log.print("  Running PWM Zero Counter test... ");
      failures = pwm_zero_counter_test(gpio_pmod0, pwm);
      test_failed |= (failures > 0);
      write_test_result(log, failures);

      log.print("  Running PWM Always High test...");
      failures = pwm_always_high_test(gpio_pmod0, pwm);
      test_failed |= (failures > 0);
      write_test_result(log, failures);
    }

    check_result(log, !test_failed);
  }
}
