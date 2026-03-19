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
% for block in config.blocks:
  % for i in range(block.instances):
  { name:  "${block.name}${f"{i}" if block.instances > 1 else "" }",
    type:  "device",
    % for (setting, value) in block.xbar.items():
    ${setting}: ${value},
    % endfor
    xbar:  false,
    addr_range: [{
      base_addr: "${"0x%08X" % (block.memory_start + i * block.memory_size)}",
      size_byte: "${"0x%08X" % (block.memory_size)}",
    }],
  },
  % endfor
% endfor
  ],
  connections: {
    ibex_lsu: [
% for block in config.blocks:
    % for i in range(block.instances):
        "${block.name}${f"{i}" if block.instances > 1 else "" }",
    % endfor
% endfor
    ],
    dbg_host: [
      "sram",
      "hyperram",
      "system_info",
    ],
  },
}


