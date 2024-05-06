SECTIONS
{
	. = 0x00100000;
	_start = .;

	.rom_loader :
	{
		*/boot.S.obj(.text);
		*.cc.obj(.text .text.* .rodata .rodata.* .data.rel.ro);
		*/end.S.obj(.text);
		. = ALIGN(4);
	}
}
