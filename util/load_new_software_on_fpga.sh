#!/bin/sh
# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

if [ $# -ne 1 ] && [ $# -ne 2 ]; then
  echo "Usage $0 elf_file [tcl_file]"
  exit 1
fi

if [ ! -f $1 ]; then
  echo "$1 does not exist"
  exit 1
fi

SCRIPT_DIR="$(dirname "$(readlink -e "$0")")"
TCL_FILE=$SCRIPT_DIR/sonata-openocd-cfg.tcl

if [ $# -eq 2 ]; then
  if [ ! -f $2 ]; then
    echo "$2 does not exist"
    exit 1
  fi
  TCL_FILE=$2
fi

openocd -f $TCL_FILE \
 -c "load_image $1 0x0" \
 -c "verify_image $1 0x0" \
 -c "exit"

