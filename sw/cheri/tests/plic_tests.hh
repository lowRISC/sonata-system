/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>
#include <functional>
#include <platform-plic.hh>
#include <platform-uart.hh>
#include "../../common/defs.h"
#include "../common/asm.hh"
#include "../common/sonata-devices.hh"
#include "../common/timer-utils.hh"
#include "test_runner.hh"

#include <array>

/**
 * Configures the number of test iterations to perform.
 * This can be overriden via a compilation flag.
 */
#ifndef PLIC_TEST_ITERATIONS
#define PLIC_TEST_ITERATIONS (1U)
#endif

struct PlicTest {
  CapRoot root;
  PLIC::SonataPlic *plic;
  Log *log_;
  uint32_t error_count = 0;

  size_t instance = 0;
  // Expected interrupt number at the PLIC.
  PLIC::Interrupts plic_irq_id;
  // Expected value of `intr_state` register in the IP block.
  // Note: this is presently necessitated by the I2C block denoting its interrupts with
  // index numbers rather than individual bits.
  uint32_t exp_intr_state;
  // IP block-local interrupt indicator.
  uint32_t ip_irq_id;

  bool is_irq_clearable;
  volatile bool irq_fired = false;

  // Rounded CPU clock cycles per microsecond, as this may not be an integral number
  static constexpr uint32_t CyclesPerUsec = (CPU_TIMER_HZ + 999'999) / 1'000'000;
  // The number of cycles to wait for an interrupt before timing out.
  static constexpr uint32_t wfiTimeout = 10 * CyclesPerUsec;

  void (*irq_handler)(PlicTest *plic_test, PLIC::Interrupts irq) = nullptr;

  void uart_test(size_t uart_instance) {
    instance = uart_instance;

    struct uart_irq {
      OpenTitanUart::OpenTitanUartInterrupt id;
      bool can_clear;
    };
    static constexpr std::array<uart_irq, 8> uartMap = {{
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveTimeout, true},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveParityErr, true},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveBreakErr, true},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveFrameErr, true},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveOverflow, true},

        {OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveWatermark, false},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptTransmitWatermark, false},
        {OpenTitanUart::OpenTitanUartInterrupt::InterruptTransmitEmpty, false},
    }};

    auto uart = uart_ptr(root, instance);

    // Ensure that initially all interrupts are cleared and disabled.
    uart->interrupt_disable(static_cast<OpenTitanUart::OpenTitanUartInterrupt>(~0u));
    uart->interruptState = ~0u;
    uart->interruptTest  = 0u;

    log_->println("  Testing UART{}:", instance);
    for (size_t i = 0; i < uartMap.size(); ++i) {
      ip_irq_id        = uartMap[i].id;
      is_irq_clearable = uartMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Uart0 + instance);
      exp_intr_state   = uartMap[i].id;

      // This lambda will handle the uart specific register to clear the irq.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        auto uart              = uart_ptr(plic_test->root, plic_test->instance);
        const bool wrong_irq   = (irq != plic_test->plic_irq_id);
        const bool wrong_local = !(uart->interruptState & plic_test->ip_irq_id);
        plic_test->error_count += wrong_irq;
        plic_test->error_count += wrong_local;
        plic_test->log(irq, uart->interruptEnable & uart->interruptState, wrong_irq || wrong_local);
        plic_test->uart_irq_clear(uart);
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      uart->interrupt_enable(uartMap[i].id);
      uart->interruptTest = ip_irq_id;
      if (!irq_caught()) {
        uart_irq_clear(uart);
      }
    }
  }

  void uart_irq_clear(UartPtr uart) {
    if (is_irq_clearable) {
      uart->interruptState = ip_irq_id;
    } else {
      // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
      uart->interruptTest = 0;
    }
    // Disable interrupt to prevent interference with other tests.
    uart->interrupt_disable(static_cast<OpenTitanUart::OpenTitanUartInterrupt>(ip_irq_id));
  }

  void i2c_test(size_t i2c_instance) {
    instance = i2c_instance;

    struct i2c_irq {
      OpenTitanI2c::Interrupt id;
      bool can_clear;
    };
    static constexpr std::array<i2c_irq, 15> i2cMap = {{
        {OpenTitanI2c::Interrupt::ReceiveOverflow, true},
        {OpenTitanI2c::Interrupt::SclInterference, true},
        {OpenTitanI2c::Interrupt::SdaInterference, true},
        {OpenTitanI2c::Interrupt::StretchTimeout, true},
        {OpenTitanI2c::Interrupt::SdaUnstable, true},
        {OpenTitanI2c::Interrupt::CommandComplete, true},
        {OpenTitanI2c::Interrupt::UnexpectedStop, true},
        {OpenTitanI2c::Interrupt::HostTimeout, true},

        {OpenTitanI2c::Interrupt::ControllerHalt, false},
        {OpenTitanI2c::Interrupt::TransmitStretch, false},
        {OpenTitanI2c::Interrupt::AcquiredFull, false},
        {OpenTitanI2c::Interrupt::TransmitThreshold, false},
        {OpenTitanI2c::Interrupt::FormatThreshold, false},
        {OpenTitanI2c::Interrupt::ReceiveThreshold, false},
        {OpenTitanI2c::Interrupt::AcquiredThreshold, false},
    }};

    auto i2c = i2c_ptr(root, instance);

    // Ensure that initially all interrupts are cleared and disabled.
    i2c->interruptEnable = 0u;
    i2c->interruptState  = ~0u;
    i2c->interruptTest   = 0u;

    log_->println("  Testing I2C{}:", instance);
    for (size_t i = 0; i < i2cMap.size(); ++i) {
      ip_irq_id        = static_cast<uint32_t>(i2cMap[i].id);
      is_irq_clearable = i2cMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::I2c0 + instance);
      exp_intr_state   = (0x1 << static_cast<uint32_t>(i2cMap[i].id));

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        auto i2c               = i2c_ptr(plic_test->root, plic_test->instance);
        const bool wrong_irq   = (irq != plic_test->plic_irq_id);
        const bool wrong_local = !(i2c->interruptState & (0x1 << plic_test->ip_irq_id));
        plic_test->error_count += wrong_irq;
        plic_test->error_count += wrong_local;
        plic_test->log(irq, i2c->interruptEnable & i2c->interruptState, wrong_irq || wrong_local);
        plic_test->i2c_irq_clear(i2c);
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      i2c->interrupt_enable(i2cMap[i].id);
      i2c->interruptTest = 0x01 << ip_irq_id;
      if (!irq_caught()) {
        i2c_irq_clear(i2c);
      }
    }
  }

  void i2c_irq_clear(I2cPtr i2c) {
    if (is_irq_clearable) {
      i2c->interruptState = 0x1 << ip_irq_id;
    } else {
      // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
      i2c->interruptTest = 0;
    }
    // Disable interrupt to prevent interference with other tests.
    i2c->interrupt_disable(static_cast<OpenTitanI2c::Interrupt>(ip_irq_id));
  }

  void spi_test(size_t spi_instance) {
    instance = spi_instance;

    struct spi_irq {
      SonataSpi::Interrupt id;
      bool can_clear;
    };
    static constexpr std::array<spi_irq, 5> spiMap = {{
        {SonataSpi::Interrupt::InterruptComplete, true},

        {SonataSpi::Interrupt::InterruptReceiveFull, false},
        {SonataSpi::Interrupt::InterruptReceiveWatermark, false},
        {SonataSpi::Interrupt::InterruptTransmitEmpty, false},
        {SonataSpi::Interrupt::InterruptTransmitWatermark, false},
    }};

    auto spi = spi_ptr(root, instance);

    // Ensure that initially all interrupts are cleared and disabled.
    spi->interrupt_disable(static_cast<SonataSpi::Interrupt>(~0u));
    spi->interruptState = ~0u;
    spi->interruptTest  = 0u;

    log_->println("  Testing SPI{}:", instance);
    for (size_t i = 0; i < spiMap.size(); ++i) {
      ip_irq_id        = spiMap[i].id;
      is_irq_clearable = spiMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Spi0 + instance);
      exp_intr_state   = spiMap[i].id;

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        auto spi               = spi_ptr(plic_test->root, plic_test->instance);
        const bool wrong_irq   = (irq != plic_test->plic_irq_id);
        const bool wrong_local = !(spi->interruptState & plic_test->ip_irq_id);
        plic_test->error_count += wrong_irq;
        plic_test->error_count += wrong_local;
        plic_test->log(irq, spi->interruptEnable & spi->interruptState, wrong_irq || wrong_local);
        plic_test->spi_irq_clear(spi);
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      spi->interrupt_enable(spiMap[i].id);
      spi->interruptTest = ip_irq_id;
      if (!irq_caught()) {
        spi_irq_clear(spi);
      }
    }
  }

  void spi_irq_clear(SpiPtr spi) {
    if (is_irq_clearable) {
      spi->interruptState = ip_irq_id;
    } else {
      // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
      spi->interruptTest = 0;
    }
    // Disable interrupt to prevent interference with other tests.
    spi->interrupt_disable(static_cast<SonataSpi::Interrupt>(ip_irq_id));
  }

  void usbdev_test() {
    instance = 0;

    struct usbdev_irq {
      OpenTitanUsbdev::UsbdevInterrupt id;
      bool can_clear;
    };
    static constexpr std::array<usbdev_irq, 18> usbdevMap = {{
        {OpenTitanUsbdev::UsbdevInterrupt::Disconnected, true},
        {OpenTitanUsbdev::UsbdevInterrupt::HostLost, true},
        {OpenTitanUsbdev::UsbdevInterrupt::LinkReset, true},
        {OpenTitanUsbdev::UsbdevInterrupt::LinkSuspend, true},
        {OpenTitanUsbdev::UsbdevInterrupt::LinkResume, true},
        {OpenTitanUsbdev::UsbdevInterrupt::AvailableBufferOverflow, true},
        {OpenTitanUsbdev::UsbdevInterrupt::LinkInError, true},
        {OpenTitanUsbdev::UsbdevInterrupt::RedundancyCheckError, true},
        {OpenTitanUsbdev::UsbdevInterrupt::PacketIdentifierError, true},
        {OpenTitanUsbdev::UsbdevInterrupt::BitstuffingError, true},
        {OpenTitanUsbdev::UsbdevInterrupt::FrameUpdated, true},
        {OpenTitanUsbdev::UsbdevInterrupt::Powered, true},
        {OpenTitanUsbdev::UsbdevInterrupt::LinkOutError, true},

        {OpenTitanUsbdev::UsbdevInterrupt::PacketReceived, false},
        {OpenTitanUsbdev::UsbdevInterrupt::PacketSent, false},
        {OpenTitanUsbdev::UsbdevInterrupt::ReceiveFull, false},
        {OpenTitanUsbdev::UsbdevInterrupt::AvailableOutEmpty, false},
        {OpenTitanUsbdev::UsbdevInterrupt::AvailableSetupEmpty, false},
    }};

    auto usbdev = usbdev_ptr(root);

    // Ensure that initially all interrupts are cleared and disabled.
    usbdev->interrupt_disable(static_cast<OpenTitanUsbdev::UsbdevInterrupt>(~0u));
    usbdev->interruptState = ~0u;
    usbdev->interruptTest  = 0u;

    log_->println("  Testing USBDEV:");
    for (size_t i = 0; i < usbdevMap.size(); ++i) {
      ip_irq_id        = static_cast<uint32_t>(usbdevMap[i].id);
      is_irq_clearable = usbdevMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Usbdev + instance);
      exp_intr_state   = static_cast<uint32_t>(usbdevMap[i].id);

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        auto usbdev            = usbdev_ptr(plic_test->root);
        const bool wrong_irq   = (irq != plic_test->plic_irq_id);
        const bool wrong_local = !(usbdev->interruptState & plic_test->ip_irq_id);
        plic_test->error_count += wrong_irq;
        plic_test->error_count += wrong_local;
        plic_test->log(irq, usbdev->interruptEnable & usbdev->interruptState, wrong_irq || wrong_local);
        plic_test->usbdev_irq_clear(usbdev);
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      usbdev->interrupt_enable(usbdevMap[i].id);
      usbdev->interruptTest = ip_irq_id;
      if (!irq_caught()) {
        usbdev_irq_clear(usbdev);
      }
    }
  }

  void usbdev_irq_clear(UsbdevPtr usbdev) {
    if (is_irq_clearable) {
      usbdev->interruptState = ip_irq_id;
    } else {
      // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
      usbdev->interruptTest = 0;
    }
    // Disable interrupt to prevent interference with other tests.
    usbdev->interrupt_disable(static_cast<OpenTitanUsbdev::UsbdevInterrupt>(ip_irq_id));
  }

  // Wait with timeout until an interrupt occurs, logging any mismatch.
  // Returns true iff the expected interrupt occurred within the timeout interval.
  inline bool irq_caught() {
    ASM::Ibex::global_interrupt_set(true);
    ASM::Ibex::global_interrupt_set(false);
    if (!irq_fired) {
      reset_mcycle();
      const uint32_t end_mcycle = get_mcycle() + wfiTimeout;
      while (get_mcycle() < end_mcycle && !irq_fired) {
        asm volatile("");
      }
    }
    if (!irq_fired) {
      error_count++;
      log(static_cast<PLIC::Interrupts>(0), 0, true);
      return false;
    }
    irq_fired = false;
    return true;
  }

  // Log the expected and observed interrupts at both the PLIC and the IP block itself;
  // the `irq_handler` function shall record any mismatches.
  void log(PLIC::Interrupts fired, uint32_t fired_intr_state, bool failed) {
    if (fired != 0 && fired_intr_state != 0) {
      log_->print("  irq fired: {:#x}, expected: {:#x} : IP-local fired: {:#x}, expected: {:#x}... ",
                  static_cast<uint32_t>(fired), static_cast<uint32_t>(plic_irq_id), fired_intr_state, exp_intr_state);
    } else {
      log_->print("  no irq fired within timeout for irq: {:#x} : IP-local: {:#x}... ",
                  static_cast<uint32_t>(plic_irq_id), exp_intr_state);
    }
    write_test_result(*log_, failed);
  }

  bool all_interrupts_test(void) {
    constexpr auto beginning = static_cast<uint32_t>(PLIC::Interrupts::Usbdev);
    constexpr auto end       = static_cast<uint32_t>(PLIC::Interrupts::MaxIntrID);

    for (uint32_t inter = beginning; inter <= end; ++inter) {
      plic->interrupt_enable(static_cast<PLIC::Interrupts>(inter));
      plic->priority_set(static_cast<PLIC::Interrupts>(inter), 1);
    }

    ASM::Ibex::external_interrupt_set(true);
    for (unsigned uart = 0; uart < UART_NUM; ++uart) {
      uart_test(uart);
    }
    for (unsigned i2c = 0; i2c < I2C_NUM; ++i2c) {
      i2c_test(i2c);
    }
    for (unsigned spi = 0; spi < SPI_NUM; ++spi) {
      spi_test(spi);
    }
    usbdev_test();
    return error_count == 0;
  }
};

PlicTest plic_test;

/**
 * Overwride the default handler
 */
extern "C" void irq_external_handler(void) {
  if (auto irq = plic_test.plic->interrupt_claim()) {
    if (plic_test.irq_handler != nullptr) {
      plic_test.irq_handler(&plic_test, *irq);
    }
    plic_test.plic->interrupt_complete(*irq);
    plic_test.irq_fired = true;
  }
}

void plic_tests(CapRoot root, Log &log) {
  PLIC::SonataPlic plic(root);
  plic_test.root = root;
  plic_test.plic = &plic;
  plic_test.log_ = &log;

  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < PLIC_TEST_ITERATIONS; i++) {
    log.println("\r\nrunning plic_test: {} \\ {}", i, PLIC_TEST_ITERATIONS - 1);
    plic_test.error_count = 0;
    check_result(log, plic_test.all_interrupts_test());
  }
}
