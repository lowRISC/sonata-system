// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "i2cdpi.hh"
#include "i2c_as621x.hh"
#include "i2c_hat_id.hh"
#include "i2c_lsm9ds1.hh"

// Bus reset signalling.
uint32_t i2cdpi::under_reset(bool scl, bool sda, uint32_t oobIn, uint32_t &oobOut) {
  state = I2C_Idle;
  numBits = 0u;
  // No active transaction.
  currDevice = nullptr;
  reading = false;
  sendAck = true;
  // Reset asserted, SCL and SDA inactive.
  prev_rst = true;
  prev_scl = true;
  prev_sda = true;
  // Note: Perhaps one or more devices may wish to emit non-zero Out-Of-Band signals under reset
  // at some point, but presently nothing uses these signals. OOB signals would classically be
  // interrupt lines and if shared they may need to default to being high.
  oobOut = 0u;
  latestOobOut = oobOut;
  // Produce outputs.
  return (prev_sda << 1) | prev_scl;
}

// Decode I2C bus signalling and produce appropriate responses.
uint32_t i2cdpi::decode(bool scl, bool sda, uint32_t oobIn, uint32_t &oobOut) {
  // Inform all devices of the end of a bus reset condition.
  if (prev_rst) {
    for (const auto &dev : devs) dev.second->busReset();
    prev_rst = false;
  }

  // Keep the Out-Of-Band output signals stabled by default.
  oobOut = latestOobOut;
  // SCL and SDA lines are pulled high when undriven; open drain bus.
  bool sda_out = true;
  // Note: We presently do not implement support for clock stretching so our contribution to the
  // SCL line is to leave it undriven (high) at all times.
  bool scl_out = true;

  switch (state) {
    case I2C_Idle:
      // Start condition?
      if (prev_sda && !sda) {
        start_cond();
        state = I2C_Addr;
        numBits = 0u;
      }
      break;

    // The first byte of a transaction consists of a 7-bit address followed by a single R/nW
    // indication of the direction of the transfer.
    case I2C_Addr:
      // SDA shall be sampled when SCL is high; respond on the rising edge.
      if (!prev_scl && scl) {
        // Collect Address and R/nW indicator; bytes are transferred MSB first.
        currByte = (currByte << 1) | (sda ? 1u : 0u);
        if (++numBits >= 8u) {
          // Attempt to locate this device; 7 MSBs of the first byte specify the
          // address of the target device.
          i2caddr_t addr = currByte >> 1;
          // Remember whether this is a read operation or a write operation.
          reading = (currByte & 1u);
          logText("Addressing target 0x%0x read %c\n", addr, reading ? 'Y' : 'N');
          currDevice = find_device(addr);
          if (currDevice) {
            // Notify the device that a transaction is being attempted.
            currDevice->accessStarting(reading);
            if (reading) {
              if (!currDevice->readByte(oobIn, currByte, oobOut)) {
                currByte = 0xffu;
              }
              // Retain these signals so that we may keep them stable.
              latestOobOut = oobOut;
            }
            sendAck = true;
          } else {
            // No target/device at the given address.
            logText("No target/device at address 0x%0x\n", addr);
            sendAck = false;
          }
          // Start counting the transmitted bits of this new byte.
          numBits = 0u;
          state = I2C_GotAddr;
        }
      }
      break;

    case I2C_GotAddr:
    case I2C_GotData:
      // Await the falling edge of SCL, and then we can change the SDA line
      // to ACK if the targeted device exists.
      if (prev_scl && !scl) {
        sda_out = !sendAck;  // ACK is indicated by pulling SDA low.
        state = I2C_SendAckNak;
      }
      break;

    case I2C_SendAckNak:
      sda_out = !currDevice;
      // Await the falling edge of SCL that indicates the ACK/NAK (9th) bit has been sampled.
      if (prev_scl && !scl) {
        state = I2C_Data;
      }
      break;

    case I2C_SentData:
      if (!prev_scl && scl) {
        // ACK/NAK response from the controller.
        bool ack = !sda;
        assert(currDevice);
        if (ack && !currDevice->readByte(oobIn, currByte, oobOut)) {
          currByte = 0xffu;
        }
        // Retain these signals so that we may keep them stable.
        latestOobOut = oobOut;
        // Start count the transmitted bits of this new byte.
        numBits = 0u;
        state = I2C_GotAckNak;
      }
      break;

    case I2C_GotAckNak:
      if (prev_scl && !scl) {
        state = I2C_Data;
      }
      break;

    case I2C_Data:
      // A rising edge on SDA whilst SCL is high indicates a stoP condition.
      if (scl && !prev_sda && sda) {
        if (currDevice) {
          currDevice->stopCond();
          currDevice = nullptr;
        }
        state = I2C_Idle;
      } else if (scl && prev_sda && !sda) {
        // A falling edge on SDA whilst SCL is high indicates a repeated start (Sr) condition.
        if (currDevice) {
          currDevice->restartCond();
          currDevice = nullptr;
        }
        state = I2C_Addr;
        numBits = 0u;
      } else if (currDevice) {
        if (reading) {
          // Emit the MS bit not yet completed; bytes are transferred Most Significant Bit first.
          sda_out = (currByte & 0x80u) != 0u;
          // When the operation is a read, we must launch the data during the interval for which
          // SCL is deasserted; this avoids transitions during the interval for which SCL is asserted.
          if (prev_scl && !scl) {
            // Shift read byte; ensure SDA returns high.
            currByte = (currByte << 1) | 1u;
            if (++numBits >= 8u) {
              state = I2C_SentData;
            }
          }
        } else {
          // When the operation is a write, the data will be stable throughout the interval for which
          // SCL is asserted; sample it on the rising edge.
          if (!prev_scl && scl) {
            currByte = (currByte << 1) | (sda ? 1u : 0u);
            if (++numBits >= 8u) {
              sendAck = currDevice->writeByte(currByte, oobIn);
              numBits = 0u;
              state = I2C_GotData;
            }
          }
        }
      }
      break;

    default:
      logText("Invalid/unexpected device state 0x%0x\n", state);
      state = I2C_Idle;
      break;
  }
  // Remember the new state of the I2C bus so that we can detect future changes.
  prev_sda = sda;
  prev_scl = scl;
  // Produce outputs.
  return (sda_out << 1) | scl_out;
}

// Logging utility function.
void i2cdpi::logText(const char *fmt, ...) {
  if (logging) {
    va_list va;
    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);
  }
}

extern "C" {
// Initialisation of DPI on the given I2C bus and attach the appropriate devices.
// Note: perhaps the bus configuration should be read from a configuration file at some point.
void *i2cdpi_create(const char *id) {
  i2cdpi *i2c = new i2cdpi();
  assert(i2c);
  // Create the appropriate devices for this bus.
  if (!strcmp(id, "i2c_rpi0")) {
    // RPI Sense HAT ID EEPROM
    const i2caddr_t id_addr = 0x50u;
    i2cdevice *id_device = new i2c_hat_id(id_addr);
    assert(id_device);
    i2c->add_device(id_device);
    i2c->logText("%s created\n", id);
  } else if (!strcmp(id, "i2c_rpi1")) {
    // RPI Sense HAT IMU; this presents at two separate I2C addresses.
    const i2caddr_t imu_accgyr_addr = 0x6au;  // Accelerometer/gyroscope.
    i2cdevice *imu_accgyr_device = new i2c_lsm9ds1(imu_accgyr_addr);
    assert(imu_accgyr_device);
    const i2caddr_t imu_mag_addr = 0x1cu;  // Magnetometer.
    i2cdevice *imu_mag_device = new i2c_lsm9ds1(imu_mag_addr);
    assert(imu_mag_device);
    i2c->add_device(imu_accgyr_device);
    i2c->add_device(imu_mag_device);
    i2c->logText("%s created\n", id);
  } else if (!strcmp(id, "i2c1")) {
    // Digital Temperature Sensor.
    const i2caddr_t dts_addr = 0x48u;
    i2cdevice *dts_device = new i2c_as621x(dts_addr);
    assert(dts_device);
    i2c->add_device(dts_device);
    i2c->logText("%s created\n", id);
  } else {
    i2c->logText("Info: I2C bus '%s' is unpopulated\n", id);
  }
  return (void *)i2c;
}

// Finalisation of I2C devices and DPI model.
void i2cdpi_destroy(void *ctx_v) {
  i2cdpi *i2c = (i2cdpi *)ctx_v;
  assert(i2c);
  delete i2c;
}

// Bus reset signalling on the given I2C bus; produce appropriate outputs.
uint32_t i2cdpi_under_reset(void *ctx_v, uint32_t sda_scl, uint32_t oobIn) {
  i2cdpi *i2c = (i2cdpi *)ctx_v;
  uint32_t oobOut;
  assert(i2c);
  sda_scl = i2c->under_reset((sda_scl & 1u) != 0u, (sda_scl & 2u) != 0u, oobIn, oobOut);
  return (oobOut << 2) | (sda_scl & 3u);
}

// Decode I2C signalling on the given bus and produce updated outputs.
uint32_t i2cdpi_decode(void *ctx_v, uint32_t sda_scl, uint32_t oobIn) {
  i2cdpi *i2c = (i2cdpi *)ctx_v;
  uint32_t oobOut;
  assert(i2c);
  sda_scl = i2c->decode((sda_scl & 1u) != 0u, (sda_scl & 2u) != 0u, oobIn, oobOut);
  return (oobOut << 2) | (sda_scl & 3u);
}
};
