// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

{ name: "main",
  type: "xbar",
  clock: "clk_sys_i", // Main clock, used in sockets
  clock_connections: {
    clk_sys_i: "main",
    clk_usb_i: "usb",
  },
  reset: "rst_sys_ni",
  reset_connections: {
    rst_sys_ni: "sys",
    rst_usb_ni: "usb",
  },
  nodes: [
    { name:  "ibex_lsu", // Load store unit
      type:  "host",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      pipeline: false,
    },
    { name:  "dbg_host", // Debug module host
      type:  "host",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      pipeline: true,
    },
    { name:  "sram", // Internal memory
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x00100000",
        size_byte: "0x00020000",
      }],
    },
    { name:  "hyperram", // HyperRAM memory
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x40000000",
        size_byte: "0x00800000",
      }],
    },
    { name:  "rev_tag", // Revocation tag memory
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x30000000",
        size_byte: "0x00000800",
      }],
    },
    { name:  "gpio", // General purpose input and output
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      addr_range: [{
        base_addr: "0x80000000",
        size_byte: "0x00001000",
      }],
      pipeline: true,
    },
    { name:  "pinmux", // Pin multiplexer
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      addr_range: [{
        base_addr: "0x80005000",
        size_byte: "0x00001000",
      }],
      pipeline: true,
    },
    { name:  "rgbled_ctrl", // RGB LED Controller
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      addr_range: [{
        base_addr: "0x80009000",
        size_byte: "0x00001000",
      }],
      pipeline: true,
    },
    { name:  "hw_rev", // Hardware revoker control register
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x8000A000",
        size_byte: "0x00001000",
      }],
    },
    { name:  "xadc", // XADC
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      addr_range: [{
        base_addr: "0x8000B000",
        size_byte: "0x00001000",
      }],
      pipeline: true,
    },
    { name:  "system_info", // System information
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x8000C000",
        size_byte: "0x00001000",
      }],
    },
    { name:  "timer", // Interrupt timer
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x80040000",
        size_byte: "0x00010000",
      }],
    },
    { name:  "spi_lcd",
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x80300000",
        size_byte: "0x00001000",
      }],
    },
    { name:  "spi_ethmac",
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x80301000",
        size_byte: "0x00001000",
      }],
    },
    % for block in config.blocks:
    % if not block.name == "gpio":
    % for i in range(block.instances):
    { name:  "${f"{block.name}{i}"}",
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "${hex(block.memory_start + i * block.memory_size)}",
        size_byte: "${hex(block.memory_size)}",
      }],
      % for (setting, value) in block.xbar.items():
      ${setting}: ${value},
      % endfor
    },
    % endfor
    % endif
    % endfor
    { name:  "usbdev", // USB device
      type:  "device",
      clock: "clk_usb_i",
      reset: "rst_usb_ni",
      xbar:  false,
      addr_range: [{
        base_addr: "0x80400000",
        size_byte: "0x00001000",
      }],
    },
    { name:     "dbg_dev", // Debug module fetch interface
      type:     "device",
      clock:    "clk_sys_i",
      reset:    "rst_sys_ni",
      xbar:     false,
      pipeline: true,
      addr_range: [{
        base_addr: "0xB0000000",
        size_byte: "0x00001000",
      }],
    },
    { name:  "rv_plic", // RISC-V platform interrupt controller
      type:  "device",
      clock: "clk_sys_i",
      reset: "rst_sys_ni",
      req_fifo_pass: false,
      rsp_fifo_pass: false,
      xbar:  false,
      addr_range: [{
        // This block is overaligned to 0x0800_0000 bytes since OpenTitan RV_PLIC block expects it.
        base_addr: "0x88000000",
        size_byte: "0x08000000",
      }],
      pipeline: true,
    },
  ],
  connections: {
    ibex_lsu: [
      "sram",
      "hyperram",
      "rev_tag",
      "dbg_dev",
      "gpio",
      "pinmux",
      "system_info",
      "rgbled_ctrl",
      "hw_rev",
      "xadc",
      "timer",
      "spi_lcd",
      "spi_ethmac",
      % for block in config.blocks:
      % if not block.name == "gpio":
      % for i in range(block.instances):
      "${f"{block.name}{i}"}",
      % endfor
      % endif
      % endfor
      "usbdev",
      "rv_plic",
    ],
    dbg_host: [
      "sram",
      "hyperram",
      "system_info",
    ],
  },
}
