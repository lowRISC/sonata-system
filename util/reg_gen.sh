#!/bin/bash
# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

declare -a regtoolblocks=("rgbled_ctrl" "spi" "system_info" "rev_ctl")

for block in "${regtoolblocks[@]}"
do
  vendor/lowrisc_ip/util/regtool.py -r -t rtl/ip/"$block"/rtl rtl/ip/"$block"/data/"$block".hjson
done
