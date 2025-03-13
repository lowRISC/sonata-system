#!/usr/bin/env sh
# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
set -ue

NAME="$(basename "$0")"
SCRIPT_DIR="$(dirname "$(readlink -e "$0")")"
TCL_FILE=$SCRIPT_DIR/sonata-openocd-cfg.tcl

USAGE="Example Usage of $NAME

  Generate an image file for an ELF file containing CHERIoT RTOS build:

    $NAME rtos_img -e \$elf_file -v \$vmem_file
    $NAME rtos_img -e \$elf_file -u \$uf2_file

  Load an ELF file onto the FPGA:

    $NAME load_program -e \$elf_file"

usage_then_exit() {
  echo "$USAGE"
  exit 2
}

check_not_empty() {
  if [ -z "$1" ]; then
    echo "$2"
    usage_then_exit
  fi
}

check_exists() {
  if [ ! -f "$1" ]; then
    echo "$2"
    usage_then_exit
  fi
}

rtos_img() {
  while getopts "e:v:u:h" opt; do
      case "${opt}" in
          e) ELF_FILE="$OPTARG";;
          v) VMEM_FILE="$OPTARG";;
          u) UF2_FILE="$OPTARG";;
          *) usage_then_exit;;
      esac
  done
  check_not_empty "${VMEM_FILE-}${UF2_FILE-}" "$NAME: No output file was given."
  check_not_empty "${ELF_FILE-}" "$NAME: No ELF file was given."
  check_exists "$ELF_FILE" "$NAME: '$ELF_FILE' doesn't exist."

  TMP_FILE="$(mktemp)"
  llvm-objcopy -O binary "$ELF_FILE" "$TMP_FILE"

  if [ -n "${VMEM_FILE-}" ]; then
    # Put the code at offset 0x80 and zero fill ibex's default vectored interrupt table
    srec_cat "$TMP_FILE" -binary -offset 0x80 -byte-swap 4 -fill 0x00 0x00 0x80 -o "$VMEM_FILE" -vmem
  fi
  if [ -n "${UF2_FILE-}" ]; then
    # Put the code at offset 0x101000 (where the bootloader jumps to after loading)
    uf2conv "$TMP_FILE" -b 0x00101000 -co "$UF2_FILE"
  fi
  rm "$TMP_FILE"
}

load_program() {
  while getopts "e:t:h" opt; do
      case "${opt}" in
          e) ELF_FILE="$OPTARG";;
          t) TCL_FILE="$OPTARG";;
          *) usage_then_exit;;
      esac
  done
  check_not_empty "${ELF_FILE-}" "$NAME: No ELF file was given."
  check_exists "$ELF_FILE" "$NAME: '$ELF_FILE' doesn't exist."
  check_exists "$TCL_FILE" "$NAME: '$TCL_FILE' doesn't exist."

  openocd -f "$TCL_FILE" \
    -c "load_image $ELF_FILE 0x0" \
    -c "verify_image $ELF_FILE 0x0" \
    -c "echo \"Doing reset\"" \
    -c "reset run" \
    -c "exit"
}

main() {
  if [ -z "${1-}" ]; then
    usage_then_exit
  fi
  case "$1" in
    rtos_img) shift && rtos_img "$@";;
    load_program) shift && load_program "$@";;
    *) usage_then_exit;;
  esac
}

main "$@"
