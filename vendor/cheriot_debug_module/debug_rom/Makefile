# See LICENSE.SiFive for license details

## Conversion to CHERIoT Ibex ISA from RISC-V
##   Copyright SCI Semiconductor 2025

debug_rom = debug_rom.sv debug_rom.dump

CXX=${CHERIOT_LLVM_ROOT}/clang++
OBJCOPY=${CHERIOT_LLVM_ROOT}/llvm-objcopy
OBJDUMP=${CHERIOT_LLVM_ROOT}/llvm-objdump
LD=${CHERIOT_LLVM_ROOT}/ld.lld

PYTHON?=python

all: $(debug_rom)

%.sv: %.img
	$(PYTHON) gen_rom.py $<

%.img: %.bin
	dd if=$< of=$@ bs=256 count=1

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.S.o: $(findstring debug_rom, $(debug_rom)).S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc $(if  $(findstring one_scratch,$@),,-DSND_SCRATCH=1) -I$(RISCV)/include $< -nostdlib -fPIC -static  -o $@

%.elf: $(findstring debug_rom, $(debug_rom)).S.o link.ld
	$(LD) --script=link.ld --relax -o $@ $<

%.dump: %.elf
	$(OBJDUMP) -d $< --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data  > $@

clean:
	rm -f *.img *.dump *.bin *.sv
