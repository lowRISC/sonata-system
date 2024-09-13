/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cdefs.h>
#include <stdint.h>

/**
 * OpenTitan USB Device
 *
 * This peripheral's source and documentation can be found at:
 * https://github.com/lowRISC/opentitan/tree/ab878b5d3578939a04db72d4ed966a56a869b2ed/hw/ip/usbdev
 *
 * Rendered register documentation is served at:
 * https://opentitan.org/book/hw/ip/uart/doc/registers.html
 */
class OpenTitanUsbdev {
 public:
  /// USBDEV supports a maximum packet length of 64 bytes.
  static constexpr uint8_t MaxPacketLen = 64U;
  /// USBDEV provides 32 buffers.
  static constexpr uint8_t NumBuffers = 32U;
  /// USBDEV supports up to 12 endpoints, in each direction.
  static constexpr uint8_t MaxEndpoints = 12U;

  /* Register definitions for the relevant parts of the OpenTitan USBDEV block; see above for
   * documentation.
   */

  /**
   * Interrupt State Register.
   */
  uint32_t intrState;
  /**
   * Interrupt Enable Register.
   */
  uint32_t intrEnable;
  /**
   * Interrupt Test Register.
   */
  uint32_t intrTest;
  /**
   * Alert Test Register.
   */
  uint32_t alertTest;
  /**
   * USB Control Register.
   */
  uint32_t usbCtrl;
  /**
   * OUT Endpoint Enable Register.
   */
  uint32_t epOutEnable;
  /**
   * IN Endpoint Enable Register.
   */
  uint32_t epInEnable;
  /**
   * USB Status Register.
   */
  uint32_t usbStat;
  /**
   * Available OUT Buffer FIFO.
   */
  uint32_t avOutBuffer;
  /**
   * Available SETUP Buffer FIFO.
   */
  uint32_t avSetupBuffer;
  /**
   * RX FIFO.
   */
  uint32_t rxFIFO;
  /**
   * SETUP Reception Enable Register.
   */
  uint32_t rxEnableSETUP;
  /**
   * OUT Reception Enable Register.
   */
  uint32_t rxEnableOUT;
  uint32_t pad0;
  /**
   * In Sent Register.
   */
  uint32_t inSent;
  /**
   * Out STALL Register.
   */
  uint32_t outStall;
  /**
   * In STALL Register.
   */
  uint32_t inStall;
  /**
   * Config IN Registers.
   */
  uint32_t configIn[MaxEndpoints];
  /**
   * Out Iso Register.
   */
  uint32_t outIso;
  /**
   * In Iso Register.
   */
  uint32_t inIso;
  /**
   * Out Data Toggle Register.
   */
  uint32_t outDataToggle;
  /**
   * In Data Toggle Register.
   */
  uint32_t inDataToggle;
  uint32_t pad1;
  uint32_t pad2;
  /**
   * PHY Config Register.
   */
  uint32_t phyConfig;

  /// USB Control Register Fields.
  static constexpr uint32_t usbCtrlEnable          = 1U;
  static constexpr uint32_t usbCtrlDeviceAddr      = 0x7F0000U;
  static constexpr unsigned usbCtrlDeviceAddrShift = 16;
  /// USB Status Register Fields.
  static constexpr uint32_t usbStatAvOutFull   = 0x800000U;
  static constexpr uint32_t usbStatRxDepth     = 0xF000000U;
  static constexpr uint32_t usbStatAvSetupFull = 0x40000000U;
  /// RX FIFO Register Fields.
  static constexpr uint32_t rxFifoBuffer    = 0x1FU;
  static constexpr uint32_t rxFifoSize      = 0x7F00U;
  static constexpr uint32_t rxFifoSetup     = 0x80000U;
  static constexpr uint32_t rxFifoEp        = 0xF00000U;
  static constexpr unsigned rxFifoSizeShift = 8U;
  static constexpr unsigned rxFifoEpShift   = 20U;
  /// Config In Register Fields.
  static constexpr uint32_t configInBuffer      = 0x1FU;
  static constexpr uint32_t configInSending     = 0x20000000U;
  static constexpr uint32_t configInPend        = 0x40000000U;
  static constexpr uint32_t configInRdy         = 0x80000000U;
  static constexpr unsigned configInBufferShift = 0U;
  static constexpr unsigned configInSizeShift   = 8U;
  /// PHY Config Register Fields.
  static constexpr uint32_t phyConfigUseDiffRcvr = 1U;

  /**
   * Ensure that the Available OUT Buffer and Available SETUP Buffers are kept supplied with
   * buffers for packet reception. `buf_avail` specifies a bitmap of the buffers that are not
   * currently committed and the return value is the updated bitmap.
   */
  [[nodiscard]] uint64_t supply_buffers(uint64_t buf_avail) volatile {
    for (uint8_t buf_num = 0U; buf_num < NumBuffers; buf_num++) {
      if (buf_avail & (1U << buf_num)) {
        if (usbStat & usbStatAvSetupFull) {
          if (usbStat & usbStatAvOutFull) {
            break;
          }
          avOutBuffer = buf_num;
        } else {
          avSetupBuffer = buf_num;
        }
        buf_avail &= ~(1U << buf_num);
      }
    }
    return buf_avail;
  }

  /**
   * Initialise the USB device, ensuring that packet buffers are available for reception and that
   * the PHY has been configured. Note that at this endpoints have not been configured and the
   * device has not been connected to the USB.
   */
  [[nodiscard]] int init(uint64_t &buf_avail) volatile {
    buf_avail = supply_buffers(((uint64_t)1U << NumBuffers) - 1U);
    phyConfig = phyConfigUseDiffRcvr;
    return 0;
  }

  /**
   * Set up the configuration of an OUT endpoint.
   */
  [[nodiscard]] int configure_out_endpoint(uint8_t ep, bool enabled, bool setup, bool iso) volatile {
    if (ep < MaxEndpoints) {
      const uint32_t epMask = 1u << ep;
      epOutEnable           = (epOutEnable & ~epMask) | (enabled ? epMask : 0u);
      rxEnableSETUP         = (rxEnableSETUP & ~epMask) | (setup ? epMask : 0U);
      rxEnableOUT           = (rxEnableOUT & ~epMask) | (enabled ? epMask : 0u);
      outIso                = (outIso & ~epMask) | (iso ? epMask : 0u);
      return 0;
    }
    return -1;
  }

  /**
   * Set up the configuration of an IN endpoint.
   */
  [[nodiscard]] int configure_in_endpoint(uint8_t ep, bool enabled, bool iso) volatile {
    if (ep < MaxEndpoints) {
      const uint32_t epMask = 1u << ep;
      epInEnable            = (epInEnable & ~epMask) | (enabled ? epMask : 0u);
      inIso                 = (inIso & ~epMask) | (iso ? epMask : 0u);
      return 0;
    }
    return -1;
  }

  /**
   * Set the STALL state of the specified endpoint pair (IN and OUT).
   */
  [[nodiscard]] int set_ep_stalling(uint8_t ep, bool stalling) volatile {
    if (ep < MaxEndpoints) {
      const uint32_t epMask = 1u << ep;
      outStall              = (outStall & ~epMask) | (stalling ? epMask : 0U);
      inStall               = (inStall & ~epMask) | (stalling ? epMask : 0U);
      return 0;
    }
    return -1;
  }

  /**
   * Connect the device to the USB, indicating its presence to the USB host controller.
   * Endpoints must already have been configured at this point because traffic may be received
   * imminently.
   */
  [[nodiscard]] int connect() volatile {
    usbCtrl = usbCtrl | usbCtrlEnable;
    return 0;
  }

  /**
   * Disconnect the device from the USB.
   */
  [[nodiscard]] int disconnect() volatile {
    usbCtrl = usbCtrl & ~usbCtrlEnable;
    return 0;
  }

  /**
   * Indicate whether the USB device is connected (pullup enabled).
   */
  [[nodiscard]] bool connected() volatile { return (usbCtrl & usbCtrlEnable) != 0; }

  /**
   * Set the device address on the USB; this address will have been supplied by the USB host
   * controller in the standard `SET_ADDRESS` Control Transfer.
   */
  [[nodiscard]] int set_device_address(uint8_t address) volatile {
    if (address < 0x80) {
      usbCtrl = (usbCtrl & ~usbCtrlDeviceAddr) | (address << usbCtrlDeviceAddrShift);
      return 0;
    }
    return -1;
  }

  /**
   * Check for and return the endpoint number and buffer number of a recently-collected IN data
   * packet. The caller is responsible for reusing or releasing the buffer.
   */
  [[nodiscard]] int packet_collected(uint8_t &ep, uint8_t &buf_num) volatile {
    uint32_t sent = inSent;
    // Clear first packet sent indication.
    for (ep = 0U; ep < MaxEndpoints; ep++) {
      uint32_t epMask = 1U << ep;
      if (sent & epMask) {
        // Clear the `in_sent` bit for this specific endpoint.
        inSent = epMask;
        // Indicate which buffer has been released.
        buf_num = (configIn[ep] & configInBuffer) >> configInBufferShift;
        return 0;
      }
    }
    return -1;
  }

  /**
   * Present a packet on the specified IN endpoint for collection by the USB host controller.
   */
  [[nodiscard]] int send_packet(uint8_t buf_num, uint8_t ep, const uint32_t *data, uint8_t size) volatile {
    // Transmission of Zero Length Packets is common over the USB.
    if (size) {
      usbdev_transfer((uint32_t *)buf_base(0x800 + buf_num * MaxPacketLen), data, size, true);
    }
    configIn[ep] = (buf_num << configInBufferShift) | (size << configInSizeShift);
    configIn[ep] = configIn[ep] | configInRdy;
    return 0;
  }

  /**
   * Test for and collect the next received packet.
   */
  [[nodiscard]] int recv_packet(uint8_t &ep, uint8_t &buf_num, uint16_t &size, bool &is_setup,
                                uint32_t *data) volatile {
    if (usbStat & usbStatRxDepth) {
      uint32_t rx = rxFIFO;  // FIFO, single word read required.

      ep       = (rx & rxFifoEp) >> rxFifoEpShift;
      size     = (rx & rxFifoSize) >> rxFifoSizeShift;
      is_setup = (rx & rxFifoSetup) != 0U;
      buf_num  = rx & rxFifoBuffer;
      // Reception of Zero Length Packets occurs in the Status Stage of IN Control Transfers.
      if (size) {
        usbdev_transfer(data, (uint32_t *)buf_base(0x800 + buf_num * MaxPacketLen), size, false);
      }
      return 0;
    }
    return -1;
  }

 private:
  /**
   * Return a pointer to the given offset within the USB device register space; this is used to
   * access the packet buffer memory.
   */
  volatile uint32_t *buf_base(uint32_t offset) volatile { return (uint32_t *)((uintptr_t)this + offset); }

  /**
   * Faster, unrolled, word-based data transfer to/from the packet buffer memory.
   */
  static void usbdev_transfer(uint32_t *dp, const uint32_t *sp, uint8_t len, bool to_dev) {
    const uint32_t *esp = (uint32_t *)((uintptr_t)sp + (len & ~15u));
    // Unrolled to mitigate the loop overheads.
    while (sp < esp) {
      dp[0] = sp[0];
      dp[1] = sp[1];
      dp[2] = sp[2];
      dp[3] = sp[3];
      dp += 4;
      sp += 4;
    }
    len &= 15u;
    // Copy the remaining whole words.
    while (len >= 4u) {
      *dp++ = *sp++;
      len -= 4u;
    }
    // Tail bytes, handling the fact that USBDEV supports only 32-bit accesses.
    if (len > 0u) {
      if (to_dev) {
        // Collect final bytes into a word.
        const uint8_t *bsp = (uint8_t *)sp;
        uint32_t d         = bsp[0];
        if (len > 1u) d |= bsp[1] << 8;
        if (len > 2u) d |= bsp[2] << 16;
        // Write the final word to the device.
        *dp = d;
      } else {
        uint8_t *bdp = (uint8_t *)dp;
        // Collect the final word from the device.
        uint32_t s = *sp;
        // Unpack it into final bytes.
        *bdp = (uint8_t)s;
        if (len > 1u) bdp[1] = (uint8_t)(s >> 8);
        if (len > 2u) bdp[2] = (uint8_t)(s >> 16);
      }
    }
  }
};
