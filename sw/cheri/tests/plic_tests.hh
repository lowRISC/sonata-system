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
    // This offset is used to compute the plic irq index of different instances.
    constexpr uint32_t instanceOffset = static_cast<uint32_t>(PLIC::Interrupts::Uart1ReceiveTimeout) -
                                        static_cast<uint32_t>(PLIC::Interrupts::Uart0ReceiveTimeout);

    struct uart_irq {
      OpenTitanUart::OpenTitanUartInterrupt id;
      bool can_clear;
    };
    constexpr std::array<std::pair<uart_irq, PLIC::Interrupts>, 7> uartMap = {{
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveTimeout, true}, PLIC::Interrupts::Uart0ReceiveTimeout},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveParityErr, true},
         PLIC::Interrupts::Uart0ReceiveParityError},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveBreakErr, true},
         PLIC::Interrupts::Uart0ReceiveBreakError},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveFrameErr, true},
         PLIC::Interrupts::Uart0ReceiveFrameError},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveOverflow, true},
         PLIC::Interrupts::Uart0ReceiveOverflow},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptReceiveWatermark, false},
         PLIC::Interrupts::Uart0ReceiveWaterMark},
        {{OpenTitanUart::OpenTitanUartInterrupt::InterruptTransmitWatermark, false},
         PLIC::Interrupts::Uart0TransmitWaterMark},
        // Fixme: TransmitEmpty interrupt is wired to the wrong index in RTL.
        //  {{OpenTitanUart::OpenTitanUartInterrupt::InterruptTransmitEmpty, false} ,
        //  PLIC::Interrupts::Uart0TransmitEmpty},
    }};

    auto uart = uart_ptr(root, instance);
    write_str(console, "testing uart: ");
    write_hex8b(console, instance);
    write_str(console, "\r\n");
    for (size_t i = 0; i < uartMap.size(); ++i) {
      ip_irq_id        = uartMap[i].first.id;
      is_irq_clearable = uartMap[i].first.can_clear;
      plic_irq_id =
          static_cast<PLIC::Interrupts>(static_cast<uint32_t>(uartMap[i].second) + (instance * instanceOffset));

      // This lambda will handle the uart specific register to clear the irq.
      irq_handler = [](PlicTest *plic_test, PLIC::Interrupts irq) {
        plic_test->log(irq);
        plic_test->error_count += (irq != plic_test->plic_irq_id);
        auto uart = uart_ptr(plic_test->root, plic_test->instance);
        // Some interrupts can't be cleared, so we have to disable it.
        if (plic_test->is_irq_clearable) {
          uart->interruptState = plic_test->ip_irq_id;
        } else {
          uart->interrupt_disable(static_cast<OpenTitanUart::OpenTitanUartInterrupt>(plic_test->ip_irq_id));
        }
      };
      uart->interrupt_enable(uartMap[i].first.id);
      uart->interruptTest = ip_irq_id;
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
    constexpr auto beginning = static_cast<uint32_t>(PLIC::Interrupts::Uart0TransmitWaterMark);
    constexpr auto end       = static_cast<uint32_t>(PLIC::Interrupts::MaxIntrID);

    for (uint32_t inter = beginning; inter < end; ++inter) {
      plic->interrupt_enable(static_cast<PLIC::Interrupts>(inter));
      plic->priority_set(static_cast<PLIC::Interrupts>(inter), 1);
    }

    ASM::Ibex::external_interrupt_set(true);
    uart_test(0);
    uart_test(1);
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
