// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Simple test code for rv_timer module.
//
// TODO: this is written in the style of the old code base for bring up,
// but hopefully in a manner that aids its adaptation to CHERIoT.

#include <cstdint>

extern "C" {
#include "sonata_system.h"
#include "timer.h"
#include "uart.h"
};

/**
 * Is the code running in simulation (as opposed to on FPGA)?
 */
#define SIMULATION 0

/**
 * Sonata RV Timer
 */
class SonataTimer
{
  // Note: register space is sparse, so a list of 'RegisterType' members
  // doesn't work very well for this class.
  typedef uint32_t RegisterType;

  // Address range.
  volatile RegisterType *base_;
  uint32_t size_;

  // Offsets of registers within address range.
  static constexpr uint32_t mtimecmp  = 0x4000u / 4;
  static constexpr uint32_t mtimecmph = mtimecmp + 1;
  static constexpr uint32_t mtime  = 0xbff8u / 4;
  static constexpr uint32_t mtimeh = mtime + 1;

public:

  volatile RegisterType *volatile &address() volatile { return base_; }
  volatile uint32_t               &bounds()  volatile { return size_; }

  // Set the current time.
  void set_time(uint64_t time) volatile {
    base_[mtime]  = 0u;
    base_[mtimeh] = (uint32_t)(time >> 32);
    base_[mtime]  = (uint32_t)time;
  }
  // Set the time comparison value.
  void set_timecmp(uint64_t time) volatile {
    base_[mtimecmp]  = UINT32_MAX;
    base_[mtimecmph] = (uint32_t)(time >> 32);
    base_[mtimecmp]  = (uint32_t)time;
  }
  // Return the current time.
  uint64_t get_time(void) volatile {
    uint32_t hi, lo;
    do {
      hi = base_[mtimeh];
      lo = base_[mtime];
    } while (hi != base_[mtimeh]);
    return ((uint64_t)hi << 32) | lo;
  }
  // Return the current time comparison value.
  uint64_t get_timecmp(void) volatile {
    uint32_t hi, lo;
    do {
      hi = base_[mtimecmph];
      lo = base_[mtimecmp];
    } while (hi != base_[mtimecmph]);
    return ((uint64_t)hi << 32) | lo;
  }

  // Return access to the singleton; alas the interrupt handling code is not
  // supplied with a reference to the timer object.
  static volatile SonataTimer &theTimer();

  // Simple interrupt handler that advances the timer comparison register.
//  static void interrupt_handler(void);
};

// Time intervals are in cycles of the system clock; we load the counter with a
// value that is already close to spilling into the upper 32 bits to make the
// test duration more practical.
const uint64_t kInitialTime     = 0xffff0000u;
#if SIMULATION
const uint64_t kFinalTime       = kInitialTime + 0x00100000u;
#else
const uint64_t kFinalTime       = kInitialTime + 0x04000000u;
#endif
const uint64_t kTriggerInterval = 0x23456u;
const uint64_t kMaxLatency      = 0x400u;
const bool kVerbose = true;

// TODO: In time this shall use Capability<>
static volatile SonataTimer timer;
volatile SonataTimer &SonataTimer::theTimer() {
   return timer;
}

// The interrupt handler function must be assigned the special attribute to ensure
// correct code generation and avoid interference with the foreground code.
static void interrupt_handler(void) __attribute__((interrupt));

// Interrupt handler does not receive an object reference/workspace pointer.
static void interrupt_handler(void) {
  volatile SonataTimer &timer = SonataTimer::theTimer();
  timer.set_timecmp(timer.get_timecmp() + kTriggerInterval);
}

int pass() {
  puts("\nTest passed");
  return 0;
}

int fail() {
  puts("\nTest failed");
  return -1;
}

//[[noreturn]] extern "C" void rom_loader_entry(void *rwRoot)
extern "C" int rom_loader_entry(void *rwRoot)
{
  timer.address() = reinterpret_cast<volatile uint32_t*>(0x80040000);
  // timer.address() = 0x80040000;
  timer.bounds()  = 0x00010000;

  install_exception_handler(7, interrupt_handler);

  // Set the timer to a defined state, without relying upon
  // a reset signal.
  uint64_t time_trigger = kInitialTime + kTriggerInterval;
  timer.set_time(kInitialTime);
  timer.set_timecmp(time_trigger);
  uint64_t time_cmp_value = timer.get_timecmp();
  if (time_cmp_value != time_trigger) {
    puthex(time_cmp_value >> 32);
    puthex(time_cmp_value);
    puts("");
    puthex(time_trigger >> 32);
    puthex(time_trigger);
    puts("");
    fail();
  }

  // Enable interrupt
  enable_interrupts(TIMER_IRQ);
  set_global_interrupt_enable(1);

  do {
    // Wait for the timer to reach the next trigger level.
    asm("wfi");
    if (timer.get_timecmp() != time_trigger) {
      // Trigger level has been moved, indicating that an interrupt has
      // occurred and been handled.
      uint64_t now = timer.get_time();
      // Check that it occurred when expected, and we've spotted it promptly.
      if (now < time_trigger || now - time_trigger > kMaxLatency) {
        puthex((uint32_t)now);
        putchar('\n');
        puthex((uint32_t)time_trigger);
        return fail();
      }
      time_trigger += kTriggerInterval;
      if (kVerbose) {
        putchar('.');
      }
    }
  } while (time_trigger < kFinalTime);

  return pass();
}

int main(void) {
#if !SIMULATION
  // This just provides some resilience against verification failures whilst
  // the code is being downloaded; temporary!
  for (unsigned i = 0; i < 0x100000; i++) {
    asm("");
  }
#endif
  // Sign-on message
  uart_init(DEFAULT_UART);
  puts("timer_test");

  return rom_loader_entry(nullptr);
}
