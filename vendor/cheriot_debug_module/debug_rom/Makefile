# See LICENSE.SiFive for license details

## Conversion to CHERIoT Ibex ISA from RISC-V
##   Copyright SCI Semiconductor 2025

debug_rom = debug_rom_rv.sv debug_rom_rv.dump debug_rom_ch.sv debug_rom_ch.dump

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

# CHERIoT debug ROM image
debug_rom_ch.o : debug_rom_ch.S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc $(if  $(findstring one_scratch,$@),,-DSND_SCRATCH=1) -I$(RISCV)/include $< -nostdlib -fPIC -static  -o $@

debug_rom_ch.elf: debug_rom_ch.o link.ld
	$(LD) --script=link.ld --relax -o $@ $<

# RISC-V debug ROM image
debug_rom_rv.o : debug_rom_rv.S
	$(CXX) -c -target riscv32-unknown-unknown -mrelax -fshort-wchar -nostdinc $(if  $(findstring one_scratch,$@),,-DSND_SCRATCH=1) -I$(RISCV)/include $< -nostdlib -fPIC -static  -o $@

debug_rom_rv.elf: debug_rom_rv.o link.ld
	$(LD) --script=link.ld --relax -o $@ $<

%.dump: %.elf
	$(OBJDUMP) -d $< --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data  > $@

clean:
	rm -f *.img *.dump *.bin *.sv *.o
