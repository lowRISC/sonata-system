# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
CFLAGS=-target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot \
	-mxcheri-rvc -mrelax -fshort-wchar -nodefaultlibs

sim_boot_stub: boot.S link.ld
	clang ${CFLAGS} -Tlink.ld -o sim_boot_stub boot.S

.PHONY: clean
clean:
	rm sim_boot_stub
