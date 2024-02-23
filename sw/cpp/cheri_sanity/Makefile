CXX=${CHERIOT_LLVM_ROOT}/clang++
LD=${CHERIOT_LLVM_ROOT}/ld.lld
OBJCOPY=${CHERIOT_LLVM_ROOT}/llvm-objcopy
OBJDUMP=${CHERIOT_LLVM_ROOT}/llvm-objdump

# Create
cpu0_irom.vhx: check-env boot.elf Makefile
	${OBJCOPY} -O binary boot.elf boot.bin
	hexdump -v -e '"%08X" "\n"' boot.bin > cpu0_irom.vhx
	srec_cat boot.bin -binary -offset 0x0000 -byte-swap 4 -o cpu0_irom.vmem -vmem

boot.elf: end.S.o boot.S.o boot.cc.o irom.ldscript Makefile
	$(LD) --script=irom.ldscript --relax -o boot.elf boot.cc.o boot.S.o end.S.o
	$(OBJDUMP) -d boot.elf > boot.dump

boot.cc.o: boot.cc Makefile
	$(CXX) -c -std=c++20 -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc -Os -g -fomit-frame-pointer -fno-builtin -fno-exceptions -fno-asynchronous-unwind-tables -fno-c++-static-destructors -fno-rtti -Werror -fvisibility=hidden -fvisibility-inlines-hidden -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/synopsis -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o boot.cc.o boot.cc -O1

boot.S.o: boot.S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc  -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/synopsis -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o boot.S.o boot.S

end.S.o: end.S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc  -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/synopsis -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o end.S.o end.S

check-env:
ifndef CHERIOT_LLVM_ROOT
	$(error CHERIOT_LLVM_ROOT is undefined)
endif
ifndef CHERIOT_RTOS_SDK
	$(error CHERIOT_RTOS_SDK is undefined)
endif

clean:
	rm -f boot.elf boot.S.o boot.cc.o cpu0_irom.vhx boot.dump boot.bin cpu0_irom.vmem end.S.o
