CAPI=2:
# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
name: "lowrisc:ip_generated:system_info"

filesets:
  files_rtl:
    depend:
    - lowrisc:tlul:adapter_reg
    files:
    - system_info_reg_pkg.sv
    - system_info_reg_top.sv
    - system_info.sv
    file_type: systemVerilogSource
targets:
  default:
    filesets:
    - files_rtl
