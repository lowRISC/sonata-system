/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "../../common/defs.h"
#include <cheri.hh>

namespace PLIC {
typedef enum : uint32_t {
  // Uart 0 Interrupts.
  Uart0TransmitWaterMark  = 1,
  Uart0ReceiveWaterMark   = 2,
  Uart0TransmitEmpty      = 3,
  Uart0ReceiveOverflow    = 4,
  Uart0ReceiveFrameError  = 5,
  Uart0ReceiveBreakError  = 6,
  Uart0ReceiveTimeout     = 7,
  Uart0ReceiveParityError = 8,
  // Uart 1 Interrupts.
  Uart1TransmitWaterMark  = 9,
  Uart1ReceiveWaterMark   = 10,
  Uart1TransmitEmpty      = 11,
  Uart1ReceiveOverflow    = 12,
  Uart1ReceiveFrameError  = 13,
  Uart1ReceiveBreakError  = 14,
  Uart1ReceiveTimeout     = 15,
  Uart1ReceiveParityError = 16,
  // I2c 0 Interrupts.
  I2c0FormatFifoThreshold  = 17,
  I2c0ReceiveFifoThreshold = 18,
  I2c0AcquireFifoThreshold = 19,
  I2c0ReceiveFifoOverflow  = 20,
  I2c0ReceiveNack          = 21,
  I2c0SclInterference      = 22,
  I2c0SdaInterference      = 23,
  I2c0StretchTimeout       = 24,
  I2c0SdaUnstable          = 25,
  I2c0CommandComplete      = 26,
  I2c0TransmitStretch      = 27,
  I2c0TransmitThreshold    = 28,
  I2c0AcquireFifoFull      = 29,
  I2c0UnexpectedStop       = 30,
  I2c0HostTimeout          = 31,
  // I2c 1 Interrupts.
  I2c1FormatFifoThreshold  = 32,
  I2c1ReceiveFifoThreshold = 33,
  I2c1AcquireFifoThreshold = 34,
  I2c1ReceiveFifoOverflow  = 35,
  I2c1ReceiveNack          = 36,
  I2c1SclInterference      = 37,
  I2c1SdaInterference      = 38,
  I2c1StretchTimeout       = 39,
  I2c1SdaUnstable          = 40,
  I2c1CommandComplete      = 41,
  I2c1TransmitStretch      = 42,
  I2c1TransmitThreshold    = 43,
  I2c1AcquiteFifoFull      = 44,
  I2c1UnexpectedStop       = 45,
  I2c1HostTimeout          = 46,
  // Ethernet Interrupts.
  Ethernet = 47,
  MaxIntrID,
} Interrupts;

enum class Priority {
  PriorityHigh,
  PriorityMedium,
  PriorityLow,
};

/**
 * Driver for the standard RISC-V Platform-Local Interrupt Controller (PLIC).
 */
class SonataPlic {
 private:
  // generic offsets according to spec
  static constexpr size_t PriorityOffset  = 0x0U;
  static constexpr size_t PendingOffset   = 0x1000U;
  static constexpr size_t EnableOffset    = 0x2000U;
  static constexpr size_t ThresholdOffset = 0x200000U;
  static constexpr size_t ClaimOffset     = 0x200004U;

  // Bounded capabilities to the individual structure fields.
  // Ideally these should be contained in one volatile struct, but
  // they are so far apart (and this lets us apply the bounds once).
  volatile uint32_t *plicPrios;
  volatile uint32_t *plicPendings;
  volatile uint32_t *plicEnables;
  volatile uint32_t *plicThres;
  volatile uint32_t *plicClaim;

 public:
  /**
   * Constructor. Initialises the interrupt controller with all interrupts disabled.
   */
  SonataPlic(CHERI::Capability<void> root) {
    constexpr auto maxIntrID    = static_cast<uint32_t>(Interrupts::MaxIntrID);
    constexpr uint32_t baseAddr = PLIC_ADDRESS;
    size_t nSources             = (maxIntrID + 31U) & ~0x1f;
    // We program the enable bits in groups of 32.
    size_t nSourcesGroups = nSources >> 5;

    auto setField = [&](auto &field, size_t offset, size_t size) {
      CHERI::Capability capability = root.cast<uint32_t>();
      capability.address()         = baseAddr + offset;
      capability.bounds()          = size;
      field                        = capability;
    };
    setField(plicPrios, PriorityOffset, nSources * sizeof(uint32_t));
    setField(plicPendings, PendingOffset, nSources / 8);
    setField(plicEnables, EnableOffset, nSources / 8);
    setField(plicThres, ThresholdOffset, sizeof(uint32_t));
    setField(plicClaim, ClaimOffset, sizeof(uint32_t));

    for (size_t i = 0; i < nSourcesGroups; i++) {
      plicEnables[i] = 0U;
    }

    for (size_t i = 1; i <= maxIntrID; i++) {
      plicPrios[i] = 0U;
    }
    // We don't make use of threshold control. Leave it at 0 so all
    // configured interrupts can fire.
    *plicThres = 0U;
  }

  /**
   * Enable the specified interrupt.
   */
  void interrupt_enable(Interrupts src) {
    auto num      = static_cast<uint32_t>(src);
    size_t idx    = num >> 5;
    size_t bitPos = num & 0x1f;

    uint32_t enables = plicEnables[idx] | (1U << bitPos);
    plicEnables[idx] = enables;
  }

  /**
   * Disable the specified interrupt.
   */
  void interrupt_disable(Interrupts src) {
    auto num      = static_cast<uint32_t>(src);
    size_t idx    = num >> 5;
    size_t bitPos = num & 0x1f;

    uint32_t enables = plicEnables[idx] & ~(1U << bitPos);
    plicEnables[idx] = enables;
  }

  /**
   * Set the priority of the specified interrupt.
   */
  void priority_set(Interrupts src, uint32_t prio) {
    plicPrios[static_cast<uint32_t>(src)] = static_cast<uint32_t>(prio);
  }

  /**
   * Fetch the interrupt number that fired and prevent it from firing.  If
   * two or more interrupts have fired then this will return the
   * highest-priority one.
   *
   * If no interrupt has fired (for example, because the interrupt line on
   * the core was raised spuriously) then this returns `stdd:nullopt`.
   */
  std::optional<Interrupts> interrupt_claim() {
    uint32_t claim = *plicClaim;
    // PLIC reserves source ID 0, which means no interrupts.
    return claim == 0 ? std::nullopt : std::optional{(Interrupts)claim};
  }

  /**
   * Tell the interrupt controller we've handled a specified interrupt ID.
   */
  void interrupt_complete(Interrupts src) { *plicClaim = static_cast<uint32_t>(src); }
};
};  // namespace PLIC
