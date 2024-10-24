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
#include "../common/uart-utils.hh"
#include "../common/asm.hh"
#include "../common/sonata-devices.hh"
#include "test_runner.hh"

#include <array>

struct PlicTest {
  CapRoot root;
  PLIC::SonataPlic *plic;
  UartPtr console;
  uint32_t error_count = 0;

  size_t instance = 0;
  PLIC::Interrupts plic_irq_id;
  uint32_t ip_irq_id;
  bool is_irq_clearable;
  volatile bool irq_fired = false;

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
    write_str(console, "testing uart: ");
    write_hex8b(console, instance);
    write_str(console, "\r\n");
    for (size_t i = 0; i < uartMap.size(); ++i) {
      ip_irq_id        = uartMap[i].id;
      is_irq_clearable = uartMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Uart0 + instance);

      // This lambda will handle the uart specific register to clear the irq.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        plic_test->log(irq);
        auto uart = uart_ptr(plic_test->root, plic_test->instance);
        plic_test->error_count += (irq != plic_test->plic_irq_id);
        plic_test->error_count += !(uart->interruptState & plic_test->ip_irq_id);
        if (plic_test->is_irq_clearable) {
          uart->interruptState = plic_test->ip_irq_id;
        } else {
          // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
          uart->interruptTest = 0;
        }
        // Disable interrupt to prevent interference with other tests.
        uart->interrupt_disable(static_cast<OpenTitanUart::OpenTitanUartInterrupt>(plic_test->ip_irq_id));
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      uart->interrupt_enable(uartMap[i].id);
      uart->interruptTest = ip_irq_id;
      wfi();
    }
  }

  void i2c_test(size_t i2c_instance) {
    instance = i2c_instance;

    struct i2c_irq {
      OpenTitanI2cInterrupt id;
      bool can_clear;
    };
    static constexpr std::array<i2c_irq, 15> i2cMap = {{
        {OpenTitanI2cInterrupt::ReceiveOverflow, true},
        {OpenTitanI2cInterrupt::SclInterference, true},
        {OpenTitanI2cInterrupt::SdaInterference, true},
        {OpenTitanI2cInterrupt::StretchTimeout, true},
        {OpenTitanI2cInterrupt::SdaUnstable, true},
        {OpenTitanI2cInterrupt::CommandComplete, true},
        {OpenTitanI2cInterrupt::UnexpectedStop, true},
        {OpenTitanI2cInterrupt::HostTimeout, true},

        {OpenTitanI2cInterrupt::ControllerHalt, false},
        {OpenTitanI2cInterrupt::TransmitStretch, false},
        {OpenTitanI2cInterrupt::AcquiredFull, false},
        {OpenTitanI2cInterrupt::TransmitThreshold, false},
        {OpenTitanI2cInterrupt::FormatThreshold, false},
        {OpenTitanI2cInterrupt::ReceiveThreshold, false},
        {OpenTitanI2cInterrupt::AcquiredThreshold, false},
    }};

    auto i2c = i2c_ptr(root, instance);
    write_str(console, "testing i2c: ");
    write_hex8b(console, instance);
    write_str(console, "\r\n");
    for (size_t i = 0; i < i2cMap.size(); ++i) {
      ip_irq_id        = static_cast<uint32_t>(i2cMap[i].id);
      is_irq_clearable = i2cMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::I2c0 + instance);

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        plic_test->log(irq);
        auto i2c = i2c_ptr(plic_test->root, plic_test->instance);
        plic_test->error_count += (irq != plic_test->plic_irq_id);
        plic_test->error_count += !(i2c->interruptState & (0x1 << plic_test->ip_irq_id));
        if (plic_test->is_irq_clearable) {
          i2c->interruptState = 0x1 << plic_test->ip_irq_id;
        } else {
          // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
          i2c->interruptTest = 0;
        }
        // Disable interrupt to prevent interference with other tests.
        i2c->interrupt_disable(static_cast<OpenTitanI2cInterrupt>(plic_test->ip_irq_id));
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      i2c->interrupt_enable(i2cMap[i].id);
      i2c->interruptTest = 0x01 << ip_irq_id;
      wfi();
    }
  }

  void spi_test(size_t spi_instance) {
    instance = spi_instance;

    struct spi_irq {
      SonataSpi::SonataSpiInterrupt id;
      bool can_clear;
    };
    static constexpr std::array<spi_irq, 5> spiMap = {{
        {SonataSpi::SonataSpiInterrupt::InterruptComplete, true},
        {SonataSpi::SonataSpiInterrupt::InterruptReceiveFull, false},

        {SonataSpi::SonataSpiInterrupt::InterruptReceiveWatermark, false},
        {SonataSpi::SonataSpiInterrupt::InterruptTransmitEmpty, false},
        {SonataSpi::SonataSpiInterrupt::InterruptTransmitWatermark, false},
    }};

    auto spi = spi_ptr(root, instance);
    write_str(console, "testing spi: ");
    write_hex8b(console, instance);
    write_str(console, "\r\n");
    for (size_t i = 0; i < spiMap.size(); ++i) {
      ip_irq_id        = spiMap[i].id;
      is_irq_clearable = spiMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Spi0 + instance);

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        plic_test->log(irq);
        plic_test->error_count += (irq != plic_test->plic_irq_id);
        auto spi = spi_ptr(plic_test->root, plic_test->instance);
        if (plic_test->is_irq_clearable) {
          spi->interruptState = plic_test->ip_irq_id;
        } else {
          // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
          spi->interruptTest = 0;
        }
        // Disable interrupt to prevent interference with other tests.
        spi->interrupt_disable(static_cast<SonataSpi::SonataSpiInterrupt>(plic_test->ip_irq_id));
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      spi->interrupt_enable(spiMap[i].id);
      spi->interruptTest = ip_irq_id;
      wfi();
    }
  }

  void usbdev_test() {
    instance = 0;

    struct usbdev_irq {
      OpenTitanUsbdev::OpenTitanUsbdevInterrupt id;
      bool can_clear;
    };
    static constexpr std::array<usbdev_irq, 18> usbdevMap = {{
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptDisconnected, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptHostLost, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptLinkReset, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptLinkSuspend, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptLinkResume, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptAvBufferOverflow, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptLinkInError, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptCrcError, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptPidError, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptBitstuffError, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptFrame, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptPowered, true},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptLinkOutError, true},

        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptPacketReceived, false},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptPacketSent, false},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptRecvFifoFull, false},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptAvOutBufferEmpty, false},
        {OpenTitanUsbdev::OpenTitanUsbdevInterrupt::InterruptAvSetupBufferEmpty, false},
    }};

    auto usbdev = usbdev_ptr(root);
    write_str(console, "testing usbdev: \r\n");
    for (size_t i = 0; i < usbdevMap.size(); ++i) {
      ip_irq_id        = usbdevMap[i].id;
      is_irq_clearable = usbdevMap[i].can_clear;
      plic_irq_id      = static_cast<PLIC::Interrupts>(PLIC::Interrupts::Usbdev + instance);

      // Register interrupt handler.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        plic_test->log(irq);
        plic_test->error_count += (irq != plic_test->plic_irq_id);
        auto usbdev = usbdev_ptr(plic_test->root);
        if (plic_test->is_irq_clearable) {
          usbdev->interruptState = plic_test->ip_irq_id;
        } else {
          // Ensure that the `intr_test` bit does not keep the Status-type interrupt asserted.
          usbdev->interruptTest = 0;
        }
        // Disable interrupt to prevent interference with other tests.
        usbdev->interrupt_disable(static_cast<OpenTitanUsbdev::OpenTitanUsbdevInterrupt>(plic_test->ip_irq_id));
      };
      // Enable and trigger the interrupt now that the handler has been registered.
      usbdev->interrupt_enable(usbdevMap[i].id);
      usbdev->interruptTest = ip_irq_id;
      wfi();
    }
  }

  inline void wfi() {
    ASM::Ibex::global_interrupt_set(true);
    ASM::Ibex::global_interrupt_set(false);
    if (!irq_fired) {
      ASM::Ibex::wfi();
      ASM::Ibex::global_interrupt_set(true);
      ASM::Ibex::global_interrupt_set(false);
    }
  }

  void log(PLIC::Interrupts fired) {
    write_str(console, "irq fired: 0x");
    write_hex8b(console, static_cast<uint32_t>(fired));
    write_str(console, ", expected: 0x");
    write_hex8b(console, static_cast<uint32_t>(plic_irq_id));
    write_str(console, "\r\n");
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

void plic_tests(CapRoot root, UartPtr console) {
  PLIC::SonataPlic plic(root);
  plic_test.root    = root;
  plic_test.plic    = &plic;
  plic_test.console = console;
  write_str(console, "running plic_test\r\n");
  check_result(console, plic_test.all_interrupts_test());
}
