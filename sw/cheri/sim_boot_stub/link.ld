SECTIONS
{
	. = 0x00100000;
	_start = .;
  .text : { *(.text) }
}
