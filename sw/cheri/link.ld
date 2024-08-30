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
		*(.vectors);
		*(.text .text.* .rodata .rodata.* .data.rel.ro);
		. = ALIGN(4);
	}

    .data :
    {
        __DATA_BEGIN__ = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        *(.sbss .sbss.*)
        *(.bss .bss.*)
        __DATA_END__ = .;
        /*
         * CHERI accesses globals using offsets from GP and the linker assumes
         * that GP is located at midpoint of the output section *for each symbol*.
         * This implies that for things to work properly all globals *must* be in
         * the same section and GP set to midpoint of that section.
         */
        __global_pointer$ = __DATA_BEGIN__ + (__DATA_END__ - __DATA_BEGIN__) / 2;
    }

    /*
     * DWARF debug sections.
     * Symbols in the DWARF debugging sections (and other note sections) are
     * relative to the beginning of the section so we begin them at 0.
     */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    .debug_pubtypes 0 : { *(.debug_pubtypes) }
    .debug_ranges   0 : { *(.debug_ranges) }

    /* Other note sections to keep */
    .comment   0 : { *(.comment) }
    .riscv.attributes   0 : { *(.riscv.attributes) }

    /*
     * Discard other sections.
     * This ensures that unknown sections are not arbitrarily placed by the linker.
     * Linker will generate error if a referenced symbol is discarded.
     */
    /DISCARD/ : { *.* }
}
