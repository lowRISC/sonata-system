/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
SECTIONS
{
	. = 0x00100000;
	_start = .;

	.rom_loader :
	{
		*(.text.start);
		*(.text .text.* .rodata .rodata.* .data.rel.ro);
		. = ALIGN(4);
	}
}
