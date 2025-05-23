# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/link.ld")
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER riscv32-unknown-elf-gcc)
set(CMAKE_OBJCOPY riscv32-unknown-elf-objcopy)
set(CMAKE_C_FLAGS_INIT
    "-march=rv32imc -mabi=ilp32 -mcmodel=medany -Wall -fvisibility=hidden -ffreestanding")

set(CMAKE_CXX_COMPILER riscv32-unknown-elf-g++)
set(CMAKE_CXX_FLAGS_INIT
    "-march=rv32imc -mabi=ilp32 -mcmodel=medany -Wall -fvisibility=hidden -ffreestanding -nostdlib -nostartfiles -fno-exceptions -fno-rtti")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_ASM_FLAGS_INIT "-march=rv32imc")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -specs=nosys.specs -T \"${LINKER_SCRIPT}\"")

