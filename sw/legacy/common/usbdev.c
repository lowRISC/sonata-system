// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>

#include "dev_access.h"
#include "usbdev.h"

/**
 * Register definitions for the relevant parts of the OpenTitan USBDEV block.
 * Distilled from https://opentitan.org/book/hw/ip/usbdev/doc/registers.html
 */

// Byte offsets of key registers.
#define USBDEV_USBCTRL        0x10U
#define USBDEV_EP_OUT_ENABLE  0x14U
#define USBDEV_EP_IN_ENABLE   0x18U
#define USBDEV_USBSTAT        0x1CU
#define USBDEV_AVOUTBUFFER    0x20U
#define USBDEV_AVSETUPBUFFER  0x24U
#define USBDEV_RXFIFO         0x28U
#define USBDEV_RXENABLE_SETUP 0x2CU
#define USBDEV_RXENABLE_OUT   0x30U
#define USBDEV_IN_SENT        0x38U
#define USBDEV_OUT_STALL      0x3CU
#define USBDEV_IN_STALL       0x40U
#define USBDEV_CONFIGIN_0     0x44U
#define USBDEV_PHY_CONFIG     0x8CU
#define USBDEV_BUFFER        0x800U

// USBCTRL register fields
#define USBDEV_CTRL_ENABLE 1U

#define USBDEV_CTRL_DEVICE_ADDRESS_SHIFT 16

// USBSTAT register fields
#define USBDEV_STAT_AV_OUT_FULL   0x00800000U
#define USBDEV_STAT_RX_DEPTH      0x0F000000U
#define USBDEV_STAT_AV_SETUP_FULL 0x40000000U

// CONFIGIN_x register fields
#define USBDEV_CONFIGIN_BUFFER  0x1FU
#define USBDEV_CONFIGIN_RDY     0x80000000U
#define USBDEV_CONFIGIN_PEND    0x40000000U
#define USBDEV_CONFIGIN_SENDING 0x20000000U

#define USBDEV_CONFIGIN_BUFFER_SHIFT 0
#define USBDEV_CONFIGIN_SIZE_SHIFT   8

// RXFIFO register fields
#define USBDEV_RXFIFO_BUFFER 0x0000001FU
#define USBDEV_RXFIFO_SIZE   0x00007F00U
#define USBDEV_RXFIFO_SETUP  0x00080000U
#define USBDEV_RXFIFO_EP     0x00F00000U

#define USBDEV_RXFIFO_SIZE_SHIFT 8
#define USBDEV_RXFIFO_EP_SHIFT  20

// PHY_CONFIG register fields
#define USBDEV_PHY_CONFIG_USE_DIFF_RCVR 1U

// USBDEV register read/write (usbdev supplies the base address implicitly).
#define USBDEV_READ(offset)        DEV_READ((usbdev->base)  + (offset))
#define USBDEV_WRITE(offset, data) DEV_WRITE((usbdev->base) + (offset), (data))

// Pointer to start of USBDEV buffer within packet buffer memory
// (usbdev supplies the base address implicitly).
#define USBDEV_BUF_START(buf) (uint32_t*)(usbdev->base + USBDEV_BUFFER + \
                                          ((buf) * USBDEV_MAX_PACKET_LEN))

// SETUP requests
typedef enum usb_setup_req {
  kUsbSetupReqGetStatus = 0,
  kUsbSetupReqClearFeature = 1,
  kUsbSetupReqSetFeature = 3,
  kUsbSetupReqSetAddress = 5,
  kUsbSetupReqGetDescriptor = 6,
  kUsbSetupReqSetDescriptor = 7,
  kUsbSetupReqGetConfiguration = 8,
  kUsbSetupReqSetConfiguration = 9,
  kUsbSetupReqGetInterface = 10,
  kUsbSetupReqSetInterface = 11,
  kUsbSetupReqSynchFrame = 12
} usb_setup_req_t;

typedef enum usb_desc_type {  // Descriptor type (wValue hi)
  kUsbDescTypeDevice = 1,
  kUsbDescTypeConfiguration,
  kUsbDescTypeString,
  kUsbDescTypeInterface,
  kUsbDescTypeEndpoint,
  kUsbDescTypeDeviceQualifier,
  kUsbDescTypeOtherSpeedConfiguration,
  kUsbDescTypeInterfacePower,
} usb_desc_type_t;

// Vendor-specific requests defined by our device/test framework
typedef enum vendor_setup_req {
  kVendorSetupReqTestConfig = 0x7C,
  kVendorSetupReqTestStatus = 0x7E
} vendor_setup_req_t;

static void usbdev_supply_buffers(usbdev_state_t *usbdev) {
  uint32_t usbstat = USBDEV_READ(USBDEV_USBSTAT);
  // Supply buffers for reception of SETUP DATA packets.
  while (!(usbstat & USBDEV_STAT_AV_SETUP_FULL)) {
    int buf = usbdev_buf_alloc(usbdev);
    if (buf < 0) {
      break;
    }
    USBDEV_WRITE(USBDEV_AVSETUPBUFFER, buf);
    usbstat = USBDEV_READ(USBDEV_USBSTAT);
  }
  // Supply buffers for reception of OUT DATA packets.
  while (!(usbstat & USBDEV_STAT_AV_OUT_FULL)) {
    int buf = usbdev_buf_alloc(usbdev);
    if (buf < 0) {
      break;
    }
    USBDEV_WRITE(USBDEV_AVOUTBUFFER, buf);
    usbstat = USBDEV_READ(USBDEV_USBSTAT);
  }
}

// Initialize the USB device.
int usbdev_init(usbdev_state_t *usbdev, uint32_t base,
                const uint8_t *dev_dscr, uint8_t dev_len,      // Device Descriptor.
                const uint8_t *cfg_dscr, uint16_t cfg_len,     // Configuration Descriptor.
                const uint8_t *test_dscr, uint8_t test_len) {  // Test Descriptor.
  // Initialize workspace.
  usbdev->base = base;
  usbdev->buf_free = (uint32_t)(((uint64_t)1u << USBDEV_NUM_BUFFERS) - 1u);
  usbdev->dev_state  = Device_Reset;
  usbdev->ctrl_state = Ctrl_Setup;

  // PHY configuration.
  USBDEV_WRITE(USBDEV_PHY_CONFIG, USBDEV_PHY_CONFIG_USE_DIFF_RCVR);

  // Set up Endpoint Zero for Control Transfers; IN, OUT and SETUP.
  usbdev_ep_config(usbdev, 0u, true, true, true);

  // Remember the locations of the device descriptor
  // and configuration descriptor.
  usbdev->dev_dscr  = dev_dscr;
  usbdev->dev_len   = dev_len;
  usbdev->cfg_dscr  = cfg_dscr;
  usbdev->cfg_len   = cfg_len;
  usbdev->test_dscr = test_dscr;
  usbdev->test_len  = test_len;

  usbdev_supply_buffers(usbdev);

  // Connect to the USB.
  USBDEV_WRITE(USBDEV_USBCTRL, USBDEV_CTRL_ENABLE);
  return 0;
}

// Finalize the USB device.
int usbdev_fin(usbdev_state_t *usbdev) {
  // Disconnect from the USB.
  USBDEV_WRITE(USBDEV_USBCTRL, 0u);
  return 0;
}

// Set endpoint configuration
int usbdev_ep_config(usbdev_state_t *usbdev, uint8_t ep, bool in, bool out, bool setup) {
  if (ep >= USBDEV_MAX_ENDPOINTS) {
    return -1;
  }

  uint32_t ep_mask = 1u << ep;
  uint32_t out_enable = USBDEV_READ(USBDEV_EP_OUT_ENABLE)  & ~ep_mask;
  uint32_t in_enable  = USBDEV_READ(USBDEV_EP_IN_ENABLE)   & ~ep_mask;
  uint32_t rxsetup_en = USBDEV_READ(USBDEV_RXENABLE_SETUP) & ~ep_mask;
  uint32_t rxout_en   = USBDEV_READ(USBDEV_RXENABLE_OUT)   & ~ep_mask;

  USBDEV_WRITE(USBDEV_EP_OUT_ENABLE,  out_enable | (out   ? ep_mask : 0u));
  USBDEV_WRITE(USBDEV_EP_IN_ENABLE,   in_enable  | (in    ? ep_mask : 0u));
  USBDEV_WRITE(USBDEV_RXENABLE_SETUP, rxsetup_en | (setup ? ep_mask : 0U));
  USBDEV_WRITE(USBDEV_RXENABLE_OUT,   rxout_en   | (out   ? ep_mask : 0u));

  return 0;
}

// Set endpoint stalling.
int usbdev_ep_stalling(usbdev_state_t *usbdev, uint8_t ep, bool stalling) {
  if (ep >= USBDEV_MAX_ENDPOINTS) {
    return -1;
  }

  uint32_t ep_mask = 1u << ep;
  uint32_t out_stall = USBDEV_READ(USBDEV_OUT_STALL)  & ~ep_mask;
  uint32_t in_stall  = USBDEV_READ(USBDEV_IN_STALL)   & ~ep_mask;

  USBDEV_WRITE(USBDEV_OUT_STALL, out_stall | (stalling ? ep_mask : 0u));
  USBDEV_WRITE(USBDEV_IN_STALL,  in_stall  | (stalling ? ep_mask : 0u));
  return 0;
}

// Return the buffer number of an available packet buffer, or -ve if all in use.
int usbdev_buf_alloc(usbdev_state_t *usbdev) {
  for (unsigned b = 0u; b < USBDEV_NUM_BUFFERS; b++) {
    uint32_t b_bit = 1u << b;
    if (usbdev->buf_free & b_bit) {
      usbdev->buf_free &= ~b_bit;
      return b;
    }
  }
  return -1;
}

// Mark the specified buffer as free for software use.
int usbdev_buf_release(usbdev_state_t *usbdev, uint8_t buf) {
  uint32_t b_bit = 1u << buf;
  if (buf >= USBDEV_NUM_BUFFERS || (usbdev->buf_free & b_bit)) {
    return -1;
  }
  usbdev->buf_free |= b_bit;   
  return 0;
}

// Faster, unrolled, word-based data transfer to/from the packet buffer memory.
static void usbdev_memcpy(uint32_t *dp, const uint32_t *sp, uint8_t len, bool to_dev) {
  const uint32_t *esp = (uint32_t*)((uintptr_t)sp + (len & ~15u));
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
      const uint8_t *bsp = (uint8_t*)sp;
      uint32_t d = bsp[0];
      if (len > 1u) d |= bsp[1] << 8;
      if (len > 2u) d |= bsp[2] << 16;
      // Write the final word to the device.
      *dp = d;
    } else {
      uint8_t *bdp = (uint8_t*)dp;
      // Collect the final word from the device.
      uint32_t s = *sp;
      // Unpack it into final bytes.
      *bdp = (uint8_t)s;
      if (len > 1u) bdp[1] = (uint8_t)(s >> 8);
      if (len > 2u) bdp[2] = (uint8_t)(s >> 16);
    }
  }
}

// Read the specified number of bytes from the given buffer.
void usbdev_buf_read(usbdev_state_t *usbdev, uint8_t *dp, uint8_t buf,
                     uint8_t len) {
  assert(!((uintptr_t)dp & 3u));
  assert(buf < USBDEV_NUM_BUFFERS);
  assert(len <= USBDEV_MAX_PACKET_LEN);
  usbdev_memcpy((uint32_t*)dp, USBDEV_BUF_START(buf), len, false);
}

// Write the specified data to the specified buffer.
void usbdev_buf_write(usbdev_state_t *usbdev, uint8_t buf, const uint8_t *sp,
                      uint8_t len) {
  assert(!((uintptr_t)sp & 3u));
  assert(buf < USBDEV_NUM_BUFFERS);
  assert(len <= USBDEV_MAX_PACKET_LEN);
  usbdev_memcpy(USBDEV_BUF_START(buf), (uint32_t*)sp, len, true);
}

// Present a data packet for collection by the USB host.
int usbdev_packet_send(usbdev_state_t *usbdev, uint8_t ep, uint8_t buf,
                       const uint8_t *sp, uint8_t len) {
  uint32_t configin_offset = USBDEV_CONFIGIN_0 + (ep << 2);

  // Retract an existing IN packet? A previous packet may be still uncollected.
  uint32_t in = USBDEV_READ(configin_offset);
  if (in & USBDEV_CONFIGIN_RDY) {
    // Cancel the existing packet by clearing the RDY bit.
    USBDEV_WRITE(configin_offset, in & ~USBDEV_CONFIGIN_RDY);
    in = USBDEV_READ(configin_offset);
    while (in & USBDEV_CONFIGIN_SENDING) {
      // In process of collecting packet; PEND or IN_SENT will become set.
      in = USBDEV_READ(configin_offset);
    }
    // Release the previous buffer
    uint8_t prev = (uint8_t)((in & USBDEV_CONFIGIN_BUFFER) >> USBDEV_CONFIGIN_BUFFER_SHIFT);
    usbdev_buf_release(usbdev, prev);
  }

  // Populate new buffer.
  usbdev_buf_write(usbdev, buf, sp, len);

  in = (len << USBDEV_CONFIGIN_SIZE_SHIFT) | (buf << USBDEV_CONFIGIN_BUFFER_SHIFT);
  USBDEV_WRITE(configin_offset, in | USBDEV_CONFIGIN_RDY);
  return 0;
}

static bool usbdev_ep0_recv(usbdev_state_t *usbdev, bool setup, uint8_t buf, uint8_t size) {
  bool release = true;
  switch (usbdev->ctrl_state) {
    case Ctrl_Setup:
      if (setup && size == 8) {
        // Received SETUP stage DATA packet.
        uint8_t _Alignas(uint32_t) data[8];
        usbdev_buf_read(usbdev, data, buf, size);
        uint16_t wValue = data[2] | (data[3] << 8);
        uint16_t wLen   = data[6] | (data[7] << 8);
        switch (data[1]) {
          // GET_DESCRIPTOR requests.
          case kUsbSetupReqGetDescriptor:
            switch (data[3]) {
              case kUsbDescTypeDevice:
                if (wLen > usbdev->dev_len) wLen = usbdev->dev_len;
                usbdev_packet_send(usbdev, 0u, buf, usbdev->dev_dscr, wLen);
                usbdev->ctrl_state = Ctrl_StatusGetDesc;
                release = false;
                break;

              case kUsbDescTypeConfiguration:
                if (wLen > usbdev->cfg_len) wLen = usbdev->cfg_len;
                usbdev_packet_send(usbdev, 0u, buf, usbdev->cfg_dscr, wLen);
                usbdev->ctrl_state = Ctrl_StatusGetDesc;
                release = false;
                break;

              default:
                usbdev_ep_stalling(usbdev, 0u, true);
                break;
            }
            break;

          // SET_ADDRESS request.
          case kUsbSetupReqSetAddress:
            usbdev->dev_addr = (uint8_t)wValue;
            usbdev_packet_send(usbdev, 0u, buf, data, 0); // Zero Length Packet as ACK.
            usbdev->ctrl_state = Ctrl_StatusSetAddr;
            release = false;
            break;

          // SET_CONFIGURATION request.
          case kUsbSetupReqSetConfiguration:
            usbdev_packet_send(usbdev, 0u, buf, data, 0); // ZLP ACK.
            usbdev->ctrl_state = Ctrl_StatusSetConfig;
            release = false;
            break;

          // Bespoke, vendor-defined Setup request to allow the USBDPI model to access the test
          // configuration.
          case kVendorSetupReqTestConfig:
            if (wLen > usbdev->test_len) wLen = usbdev->test_len;
            usbdev_packet_send(usbdev, 0u, buf, usbdev->test_dscr, wLen);
            usbdev->ctrl_state = Ctrl_StatusGetDesc;
            release = false;
            break;

          default:
            usbdev->ctrl_state = Ctrl_Setup;
            break;
        }
      }
      break;

    case Ctrl_StatusGetDesc:
    default:
      usbdev->ctrl_state = Ctrl_Setup;
      break;
  }
  return release;
}

static bool usbdev_ep0_sent(usbdev_state_t *usbdev, uint8_t buf) {
  switch (usbdev->ctrl_state) {
    case Ctrl_StatusSetAddr:
      usbdev->dev_state = Device_Addressed;
      USBDEV_WRITE(USBDEV_USBCTRL, (usbdev->dev_addr << USBDEV_CTRL_DEVICE_ADDRESS_SHIFT) | USBDEV_CTRL_ENABLE);
      usbdev->ctrl_state = Ctrl_Setup;
      break;

    case Ctrl_StatusSetConfig:
      usbdev->dev_state = Device_Configured;
      usbdev->ctrl_state = Ctrl_Setup;
      break;

    default:
      usbdev->ctrl_state = Ctrl_Setup;
      break;
  }
  return true;
}

static void usbdev_service_sending(usbdev_state_t *usbdev) {
  // Any packets sent?
  // - these buffers may be released immediately.
  uint32_t sent = USBDEV_READ(USBDEV_IN_SENT);
  if (sent) {
    unsigned ep = 0u;
    USBDEV_WRITE(USBDEV_IN_SENT, sent);

    while (sent && ep < USBDEV_MAX_ENDPOINTS) {
      uint32_t ep_bit = 1u << ep;
      if (sent & ep_bit) {
        uint32_t in = USBDEV_READ(USBDEV_CONFIGIN_0 + (ep << 2));
        uint8_t buf = in & USBDEV_CONFIGIN_BUFFER;
        bool release = true;
        if (!ep) {
          // Service Endpoint Zero (Default Control Pipe)
          release = usbdev_ep0_sent(usbdev, buf);
        }
        if (release) {
          usbdev_buf_release(usbdev, buf);
        }
        sent &= ~ep_bit;
      }
      ep++;
    }
  }
}

static void usbdev_service_recving(usbdev_state_t *usbdev) {
  // Any packets received?
  if (USBDEV_READ(USBDEV_USBSTAT) & USBDEV_STAT_RX_DEPTH) {
    // Collect packet properties from RX FIFO.
    uint32_t rxfifo = USBDEV_READ(USBDEV_RXFIFO);
    bool setup = (rxfifo & USBDEV_RXFIFO_SETUP) != 0u;
    uint8_t buf = rxfifo & USBDEV_RXFIFO_BUFFER;
    uint8_t size = (rxfifo & USBDEV_RXFIFO_SIZE) >> USBDEV_RXFIFO_SIZE_SHIFT;
    uint8_t ep = (rxfifo & USBDEV_RXFIFO_EP) >> USBDEV_RXFIFO_EP_SHIFT;
    bool release = true;
    if (!ep) {
      release = usbdev_ep0_recv(usbdev, setup, buf, size);
    }
    if (release) {
      usbdev_buf_release(usbdev, buf);
    }
  }
}

int usbdev_service(usbdev_state_t *usbdev) {
  // Service SENT buffers first, because this can make more buffers available
  // for use.
  usbdev_service_sending(usbdev);
  // Ensure that we keep the Available OUT/SETUP buffer FIFOS topped up.
  usbdev_supply_buffers(usbdev);
  // Handle any received packets.
  usbdev_service_recving(usbdev);
  return 0;
}

// Indicate whether the USB device is connected, configured and capable of
// transferring data.
bool usbdev_active(usbdev_state_t *usbdev) {
  return (usbdev->dev_state == Device_Configured);
}

