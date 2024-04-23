// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef __USBDEV_H__
#define __USDBEV_H__
#include <stdbool.h>
#include <stdint.h>

// USBDEV supports a maximmum packet length of 64 bytes.
#define USBDEV_MAX_PACKET_LEN 64U
// USBDEV provides 32 buffers.
#define USBDEV_NUM_BUFFERS    32U
// USBDEV supports up to 12 endpoints, in each direction.
#define USBDEV_MAX_ENDPOINTS  12U

/***********************************************************************/
/* Below this point are macros used to construct the USB configuration */
/* descriptor. Use them to initialize a uint8_t array for cfg_dscr     */

#define USB_CFG_DSCR_LEN 9
#define USB_CFG_DSCR_HEAD(total_len, nint)                                   \
  /* This is the actual configuration descriptor                 */          \
  USB_CFG_DSCR_LEN,     /* bLength                                   */      \
      2,                /* bDescriptorType                           */      \
      (total_len)&0xff, /* wTotalLength[0]                           */      \
      (total_len) >> 8, /* wTotalLength[1]                           */      \
      (nint),           /* bNumInterfaces                            */      \
      1,                /* bConfigurationValue                       */      \
      0,                /* iConfiguration                            */      \
      0xC0,             /* bmAttributes: must-be-one, self-powered   */      \
      50 /* bMaxPower                                 */ /* MUST be followed \
                                                            by (nint)        \
                                                            Interface +      \
                                                            Endpoint         \
                                                            Descriptors */

// KEEP BLANK LINE ABOVE, it is in the macro!

#define USB_INTERFACE_DSCR_LEN 9
#define VEND_INTERFACE_DSCR(inum, nep, subclass, protocol)               \
  /* interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12 */   \
  USB_INTERFACE_DSCR_LEN, /* bLength                             */      \
      4,                  /* bDescriptorType                     */      \
      (inum),             /* bInterfaceNumber                    */      \
      0,                  /* bAlternateSetting                   */      \
      (nep),              /* bNumEndpoints                       */      \
      0xff,               /* bInterfaceClass (Vendor Specific)   */      \
      (subclass),         /* bInterfaceSubClass                  */      \
      (protocol),         /* bInterfaceProtocol                  */      \
      0 /* iInterface                          */ /* MUST be followed by \
                                                     (nep) Endpoint      \
                                                     Descriptors */

// KEEP BLANK LINE ABOVE, it is in the macro!

#define USB_EP_DSCR_LEN 7
#define USB_EP_DSCR(in, ep, attr, maxsize, interval)                          \
  /* endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13   */       \
  USB_EP_DSCR_LEN,                 /* bLength                              */ \
      5,                           /* bDescriptorType                      */ \
      (ep) | (((in) << 7) & 0x80), /* bEndpointAddress, top bit set for IN */ \
      attr,                        /* bmAttributes                         */ \
      (maxsize)&0xff,              /* wMaxPacketSize[0]                    */ \
      (maxsize) >> 8,              /* wMaxPacketSize[1]                    */ \
      (interval)                   /* bInterval                            */

// KEEP BLANK LINE ABOVE, it is in the macro!
#define USB_BULK_EP_DSCR(in, ep, maxsize, interval)                           \
  /* endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13   */       \
  USB_EP_DSCR_LEN,                 /* bLength                              */ \
      5,                           /* bDescriptorType                      */ \
      (ep) | (((in) << 7) & 0x80), /* bEndpointAddress, top bit set for IN */ \
      0x02,                        /* bmAttributes (0x02=bulk, data)       */ \
      (maxsize)&0xff,              /* wMaxPacketSize[0]                    */ \
      (maxsize) >> 8,              /* wMaxPacketSize[1]                    */ \
      (interval)                   /* bInterval                            */

// KEEP BLANK LINE ABOVE, it is in the macro!


// Device state
typedef enum {
  Device_Reset,
  Device_Addressed,
  Device_Configured
} usbdev_dev_state_t;

// Control Transfer state
typedef enum {
  Ctrl_Setup,
  Ctrl_StatusSetAddr,
  Ctrl_StatusGetDesc,
  Ctrl_StatusSetConfig
} usbdev_ctrl_state_t;

// State information for the USB device
typedef struct {
  // Base address of USBDEV hardware.
  uint32_t       base;
  // Bitmap of buffers presently available for software use.
  uint32_t       buf_free;

  // Properties of Device Descriptor
  const uint8_t *dev_dscr;
  uint8_t        dev_len;

  // Properties of Configuration Descriptor
  const uint8_t *cfg_dscr;
  uint16_t       cfg_len;

  uint8_t        dev_addr;

  // Device state
  usbdev_dev_state_t  dev_state;
  // Control Transfer handling for Default Control Pipe
  usbdev_ctrl_state_t ctrl_state;

} usbdev_state_t;


// Initialize the USB device.
int usbdev_init(usbdev_state_t *usbdev, uint32_t base,
                const uint8_t *dev_dscr, uint8_t dev_len,  // Device Descriptor
                const uint8_t *cfg_dscr, uint8_t cfg_len); // Config Descriptor

// Finalize the USB device.
int usbdev_fin(usbdev_state_t *usb);

// Set endpoint configuration.
int usbdev_ep_config(usbdev_state_t *usb, uint8_t ep, bool in, bool out, bool setup);

// Set endpoint stalling.
int usbdev_ep_stalling(usbdev_state_t *usb, uint8_t ep, bool stall);

// Return the buffer number of an available packet buffer, or -ve if all in use.
int usbdev_buf_alloc(usbdev_state_t *usbdev);

// Read the specified number of bytes from the given buffer.
void usbdev_buf_read(usbdev_state_t *usbdev, uint8_t *dp, uint8_t buf,
                     uint8_t len);

// Write the specified data to the specified buffer.
void usbdev_buf_write(usbdev_state_t *usb, uint8_t buf, const uint8_t *sp,
                      uint8_t len);

// Present a data packet for collection by the USB host.
int usbdev_packet_send(usbdev_state_t *usbdev, uint8_t ep, uint8_t buf,
                       const uint8_t *sp, uint8_t len);

// Collect a SETUP/OUT data packet from the USB device.
int usbdev_packet_recv(void);

int usbdev_service(usbdev_state_t *usbdev);

// Indicate whether the USB device is connected, configured and capable of
// transferring data.
bool usbdev_active(usbdev_state_t *usbdev);

#endif

