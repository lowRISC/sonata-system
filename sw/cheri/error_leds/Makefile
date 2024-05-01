CXX=${CHERIOT_LLVM_ROOT}/clang++
LD=${CHERIOT_LLVM_ROOT}/ld.lld
OBJCOPY=${CHERIOT_LLVM_ROOT}/llvm-objcopy
OBJDUMP=${CHERIOT_LLVM_ROOT}/llvm-objdump

# Create
cpu0_irom.vhx: check-env error.elf Makefile
	${OBJCOPY} -O binary error.elf error.bin
	hexdump -v -e '"%08X" "\n"' error.bin > cpu0_irom.vhx
	srec_cat error.bin -binary -offset 0x0000 -byte-swap 4 -o cpu0_irom.vmem -vmem

error.elf: end.S.o error.S.o irom.ldscript Makefile
	$(LD) --script=irom.ldscript --relax -o error.elf error.S.o end.S.o
	$(OBJDUMP) -d error.elf > error.dump

error.S.o: error.S
	$(CXX) -c -target riscv32-unknown-unknown -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar -nostdinc  -I${CHERIOT_RTOS_SDK}/include/c++-config -I${CHERIOT_RTOS_SDK}/include/libc++ -I${CHERIOT_RTOS_SDK}/include -I${CHERIOT_RTOS_SDK}/include/platform/arty-a7 -I${CHERIOT_RTOS_SDK}/include/platform/synopsis -I${CHERIOT_RTOS_SDK}/include/platform/ibex -I${CHERIOT_RTOS_SDK}/include/platform/generic-riscv -DIBEX -DIBEX_SAFE -DCPU_TIMER_HZ=20000000 -o error.S.o error.S

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
	rm -f error.elf error.S.o cpu0_irom.vhx error.dump error.bin cpu0_irom.vmem end.S.o
