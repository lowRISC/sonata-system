/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <ctype.h>
#include <cheri.hh>
#include <stdint.h>

#include "platform-usbdev.hh"

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

/***********************************************************************/
/* Below this point are macros used to construct the test descriptor   */
/* Use them to initialize a uint8_t array for test_dscr                */
#define USB_TESTUTILS_TEST_DSCR_LEN 0x10u
#define USB_TESTUTILS_TEST_DSCR(num, arg0, arg1, arg2, arg3)            \
  0x7e, 0x57, 0xc0, 0xf1u,                /* Header signature        */ \
      (USB_TESTUTILS_TEST_DSCR_LEN)&0xff, /* Descriptor length[0]    */ \
      (USB_TESTUTILS_TEST_DSCR_LEN) >> 8, /* Descriptor length[1]    */ \
      (num)&0xff,                         /* Test number[0]          */ \
      (num) >> 8,                         /* Test number[1]          */ \
      (arg0), (arg1), (arg2), (arg3),     /* Test-specific arugments */ \
      0x1fu, 0x0cu, 0x75, 0xe7u           /* Tail signature */

// KEEP BLANK LINE ABOVE, it is in the macro!

/// Utility functions and state information about the USB device and the configuration sequence.
///
/// This class provides the implementation of the Default Control Pipe and communicates with other
/// registered endpoints via callback functions.
class UsbdevUtils
{
  public:
    UsbdevUtils(CHERI::Capability<volatile OpenTitanUsbdev>& dev,
                const uint8_t *device, uint8_t device_len, // Device Descriptor.
                const uint8_t *cfg, uint16_t cfg_len,      // Configuration Descriptor.
                const uint8_t *test, uint8_t test_len) :   // Test Descriptor.
      usbdev(dev),
      devAddr(0u),
      devState(Device_Powered),
      ctrlState(Ctrl_Setup),
      devDscr(device),
      devLen(device_len),
      cfgDscr(cfg),
      cfgLen(cfg_len),
      testDscr(test),
      testLen(test_len),
      bufAvail(((uint64_t)1u << OpenTitanUsbdev::NumBuffers) - 1u)
   {
      // Initialise the device and track which packet buffers are still available for use.
      int rc = usbdev->init(bufAvail);
      assert(!rc);
      // Set up the Default Control Pipe.
      // The USB host controller uses this endpoint to inspect and configure the device.
      bool ok = setup_out_endpoint(0u, true, true, false, ep0_recv_cb, this);
      if (ok) ok = setup_in_endpoint(0u, true, false, ep0_sent_cb, this);
      assert(ok);
      // Ensure that the remaining endpoints have defined state.
      for (uint8_t ep = 1u; ep < OpenTitanUsbdev::MaxEndpoints; ++ep)
      {
        epOutCtx[ep].recvCallback  = nullptr;
        epInCtx[ep].txDoneCallback = nullptr;
      }
      // Ensure buffers are available for packet reception.
      supply_buffers();
    }

    /// Connect the device to the USB.
    bool connect()
    {
      int rc = usbdev->connect();
      if (rc) return false;
      devState = Device_Powered;
      return true;
    }

    /// Disconnect the device from the USB.
    bool disconnect()
    {
      int rc = usbdev->disconnect();
      if (rc) return false;
      devState = Device_Powered;
      return true;
    }

    /// Indicate whether the device is connected to the USB (pullup enabled).
    bool connected()
    {
      return usbdev->connected();
    }

    /// Allocate a buffer.
    bool buf_alloc(uint8_t &bufNum)
    {
      if (bufAvail) {
        for (bufNum = 0U; bufNum < OpenTitanUsbdev::NumBuffers; ++bufNum) {
          if (1U & (bufAvail >> bufNum)) {
            bufAvail &= ~((uint64_t)1U << bufNum);
            return true;
          }
        }
      }
      return false;
    }

    /// Release a buffer.
    void buf_release(uint8_t bufNum)
    {
      // Try to make the buffer available for reception; the USB device can only have ownership of
      // 20 buffers simultaneously on the reception side there are 32 buffers available.
      bufAvail = usbdev->supply_buffers(bufAvail | ((uint64_t)1U << bufNum));
    }

    /// Ensure that the USB device remains supplied with buffers for packet reception.
    void supply_buffers()
    {
      // Ensure that we keep buffers available for OUT packet reception.
      bufAvail = usbdev->supply_buffers(bufAvail);
    }

    // Packet reception callback handler.
    typedef void (*UsbdevRecvCB)(void *handle, uint8_t ep, bool setup, const uint8_t *data,
                                 uint16_t pktLen);

    // Transmission done callback handler.
    typedef void (*UsbdevDoneCB)(void *handle, int rc);

    /// Configure an OUT endpoint within the USB device; packet reception invokes the supplied
    /// callback function.
    bool setup_out_endpoint(uint8_t ep, bool enabled, bool setup, bool iso, UsbdevRecvCB callback,
                            void *handle)
    {
      int rc = usbdev->configure_out_endpoint(ep, enabled, setup, iso);
      if (rc) return false;
      // Remember the callback function for this endpoint and its handle.
      epOutCtx[ep].recvCallback = callback;
      epOutCtx[ep].recvHandle = handle;
      return true;
    }

    /// Configure an IN endpoint within the USB device; packet collection from this endpoint results
    /// in the supplied callback function being invoked.
    bool setup_in_endpoint(uint8_t ep, bool enabled, bool iso, UsbdevDoneCB callback, void *handle)
    {
      int rc = usbdev->configure_in_endpoint(ep, enabled, iso);
      if (rc) return false;
      // Remember the callback function or this endpoint and its handle.
      epInCtx[ep].txDoneCallback = callback;
      epInCtx[ep].txDoneHandle = handle;
      return true;
    }

    /// Has the device been configured by the USB host controller?
    bool configured() const { return devState == Device_Configured; }

    /// Send a packet of data over the USB to the host controller.
    bool send_data(uint8_t ep, const uint32_t *data, uint16_t pktLen)
    {
      uint8_t bufNum;
      assert(pktLen <= OpenTitanUsbdev::MaxPacketLen);
      if (!buf_alloc(bufNum)) return false;
      if (usbdev->send_packet(bufNum, ep, data, (uint8_t)pktLen))
      {
        buf_release(bufNum);
        return false;
      }
      return true;
    }

    /// Service the USB device; this must be called regularly in order to minimise the latency of
    /// responses to the USB host controller.
    void service()
    {
      // Process the completion of any IN packets that have been collected by the USB host
      // controller; this must be done promptly not just for performance reasons but also to ensure
      // that IN Control Transfers conclude their Data Stage before the Status Stage commences.
      uint8_t ep, bufNum;
      int rc = usbdev->packet_collected(ep, bufNum);
      while (!rc)
      {
        // Release the buffer for reuse.
        buf_release(bufNum);
        if (epInCtx[ep].txDoneCallback)
        {
          // Invoke the 'done' handler for this IN endpoint.
          epInCtx[ep].txDoneCallback(epInCtx[ep].txDoneHandle, rc);
        }
        rc = usbdev->packet_collected(ep, bufNum);
      }

      // Ensure that the packet reception FIFOs remains supplied with buffers.
      supply_buffers();

      // Process received packets.
      uint16_t pktLen;
      bool isSetup;
      rc = usbdev->recv_packet(ep, bufNum, pktLen, isSetup, packetData);
      while (!rc)
      {
        // Release the buffer for reuse.
        buf_release(bufNum);
        if (epOutCtx[ep].recvCallback)
        {
          const uint8_t *pktData = reinterpret_cast<uint8_t *>(packetData);
          epOutCtx[ep].recvCallback(epOutCtx[ep].recvHandle, ep, isSetup, pktData, pktLen);
        }
        rc = usbdev->recv_packet(ep, bufNum, pktLen, isSetup, packetData);
      }
    }

private:
    /// Packet reception callback handler for endpoint zero.
    static void ep0_recv_cb(void *handle, uint8_t ep, bool setup, const uint8_t *pktData,
                            uint16_t pktLen)
    {
      UsbdevUtils *utils = reinterpret_cast<UsbdevUtils *>(handle);
      utils->ep0_recv(ep, setup, pktData, pktLen);
    }
    /// Transmission done callback handler for endpoint zero.
    static void ep0_sent_cb(void *handle, int rc)
    {
      UsbdevUtils *utils = reinterpret_cast<UsbdevUtils *>(handle);
      utils->ep0_sent(rc);
    }

    /// Process the reception of a packet on the Default Control Pipe (Endpoint Zero).
    void ep0_recv(uint8_t ep, bool setup, const uint8_t *data, uint16_t pktLen)
    {
      // Do we need to release the buffer?
      bool release = true;
      // Allocate a buffer for the reply.
      uint8_t bufNum;
      int rc;
      bool ok = buf_alloc(bufNum);
      assert(ok);
      switch (ctrlState)
      {
        case Ctrl_Setup:
          if (setup && pktLen == 8u)
          {
            // Received SETUP stage DATA packet.
            uint16_t wValue = data[2] | (data[3] << 8);
            uint16_t wLen   = data[6] | (data[7] << 8);
            switch (data[1])
            {
              // GET_DESCRIPTOR requests.
              case kUsbSetupReqGetDescriptor:
                switch (data[3])
                {
                  case kUsbDescTypeDevice:
                    if (wLen > devLen) wLen = devLen;
                    rc = usbdev->send_packet(bufNum, 0u, (uint32_t*)devDscr, (uint8_t)wLen);
                    assert(!rc);
                    ctrlState = Ctrl_StatusGetDesc;
                    release = false;
                    break;

                  case kUsbDescTypeConfiguration:
                    if (wLen > cfgLen) wLen = cfgLen;
                    rc = usbdev->send_packet(bufNum, 0u, (uint32_t*)cfgDscr, (uint8_t)wLen);
                    assert(!rc);
                    ctrlState = Ctrl_StatusGetDesc;
                    release = false;
                    break;

                  default:
                    rc = usbdev->set_ep_stalling(0u, true);
                    assert(!rc);
                    break;
                }
                break;

              // SET_ADDRESS request.
              case kUsbSetupReqSetAddress:
                devAddr = (uint8_t)wValue;
                // Send a Zero Length Packet as ACK.
                rc = usbdev->send_packet(bufNum, 0u, nullptr, 0u);
                assert(!rc);
                ctrlState = Ctrl_StatusSetAddr;
                release = false;
                break;

              // SET_CONFIGURATION request.
              case kUsbSetupReqSetConfiguration:
                rc = usbdev->send_packet(bufNum, 0u, nullptr, 0u); // ZLP ACK.
                assert(!rc);
                ctrlState = Ctrl_StatusSetConfig;
                release = false;
                break;

              // Bespoke, vendor-defined Setup request to allow the USBDPI model to access the test
              // configuration.
              case kVendorSetupReqTestConfig:
                if (wLen > testLen) wLen = testLen;
                rc = usbdev->send_packet(bufNum, 0u, (uint32_t*)testDscr, (uint8_t)wLen);
                assert(!rc);
                ctrlState = Ctrl_StatusGetDesc;
                release = false;
                break;

              default:
                ctrlState = Ctrl_Setup;
                break;
            }
          }
          break;

        case Ctrl_StatusGetDesc:
        default:
          ctrlState = Ctrl_Setup;
          break;
      }
      // Release the buffer if it has not been used for a reply.
      if (release) buf_release(bufNum);
    }

    /// Process the collection of an IN packet on the Default Control Pipe (Endpoint Zero).
    void ep0_sent(int rc)
    {
      switch (ctrlState)
      {
        case Ctrl_StatusSetAddr:
          devState = Device_Addressed;
          rc = usbdev->set_device_address(devAddr);
          assert(!rc);
          ctrlState = Ctrl_Setup;
          break;

        case Ctrl_StatusSetConfig:
          devState = Device_Configured;
          ctrlState = Ctrl_Setup;
          break;

        default:
          ctrlState = Ctrl_Setup;
          break;
      }
    }

  private:
    // Access to the USB device driver.
    CHERI::Capability<volatile OpenTitanUsbdev> usbdev;

    // Assigned device address on the USB.
    uint8_t devAddr;

    // Device state
    enum DeviceState {
      Device_Powered,
      Device_Addressed,
      Device_Configured
    } devState;

    // Control Transfer state
    enum CtrlState {
      Ctrl_Setup,
      Ctrl_StatusSetAddr,
      Ctrl_StatusGetDesc,
      Ctrl_StatusSetConfig
    } ctrlState;

    // Properties of Device Descriptor.
    const uint8_t *devDscr;
    uint8_t devLen;
    // Properties of Configuration Descriptor.
    const uint8_t *cfgDscr;
    uint16_t cfgLen;
    // Properties of Test Descriptor.
    const uint8_t *testDscr;
    uint8_t testLen;

    // Bitmap of available packet buffers.
    uint64_t bufAvail;

    // Context for IN endpoints.
    struct
    {
      // Callback handler function for packet transmissions that have completed.
      UsbdevDoneCB txDoneCallback;
      // Tx done handle.
      void  *txDoneHandle;
    } epInCtx[OpenTitanUsbdev::MaxEndpoints];

    // Context for OUT endpoints.
    struct
    {
      // Callback handler function for received packets.
      UsbdevRecvCB recvCallback;
      // Callback handle.
      void  *recvHandle;
    } epOutCtx[OpenTitanUsbdev::MaxEndpoints];

    // Buffer for received packet data.
    uint32_t packetData[OpenTitanUsbdev::MaxPacketLen >> 2];
};
