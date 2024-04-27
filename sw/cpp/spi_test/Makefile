CXX=${CHERIOT_LLVM_ROOT}/clang++
LD=${CHERIOT_LLVM_ROOT}/ld.lld
OBJCOPY=${CHERIOT_LLVM_ROOT}/llvm-objcopy
OBJDUMP=${CHERIOT_LLVM_ROOT}/llvm-objdump

# Create
cpu0_irom.vhx: check-env spi_test.elf Makefile
	${OBJCOPY} -O binary spi_test.elf spi_test.bin
	hexdump -v -e '"%08X" "\n"' spi_test.bin > cpu0_irom.vhx
	srec_cat spi_test.bin -binary -offset 0x0000 -byte-swap 4 -o cpu0_irom.vmem -vmem

%.elf: %.cc.o end.S.o boot.S.o irom.ldscript Makefile
	$(LD) --script=irom.ldscript --relax -o $@ $< boot.S.o end.S.o
	$(OBJDUMP) -dlS $@ > $@.dump

%.cc.o: %.cc Makefile
	$(CXX) -c -std=c++20 -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc -Os -g -fomit-frame-pointer -fno-builtin -fno-exceptions -fno-asynchronous-unwind-tables -fno-c++-static-destructors -fno-rtti -Wall -Werror -fvisibility=hidden -fvisibility-inlines-hidden -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o $@ $< -O1

%.S.o: %.S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc  -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o $@ $<

check-env:
ifndef CHERIOT_LLVM_ROOT
	$(error CHERIOT_LLVM_ROOT is undefined)
endif
ifndef CHERIOT_RTOS_SDK
	$(error CHERIOT_RTOS_SDK is undefined)
endif

clean:
	rm -f *.elf *.S.o *.cc.o cpu0_irom.vhx *.dump *.bin cpu0_irom.vmem
