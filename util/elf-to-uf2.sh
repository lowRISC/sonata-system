#!/bin/sh
# Copyright lowRISC Contributors.
# SPDX-License-Identifier: Apache-2.0
set -ue
llvm-strip "$1" -o "$1.strip"
uf2conv "$1.strip" -f0x6CE29E60 -co "$1.uf2"
