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
