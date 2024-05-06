set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/link.ld")

set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_ASM_COMPILER clang)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_ASM_COMPILER_TARGET riscv32-unknown-unknown)
set(CMAKE_C_COMPILER_TARGET riscv32-unknown-unknown)
set(CMAKE_CXX_COMPILER_TARGET riscv32-unknown-unknown)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

string(CONCAT CMAKE_CXX_FLAGS_INIT
  "-std=c++20 -mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar"
  " -Os -g -fomit-frame-pointer -fno-builtin -fno-exceptions"
  " -fno-asynchronous-unwind-tables -fno-c++-static-destructors -fno-rtti"
  " -Wall -Werror -fvisibility=hidden -fvisibility-inlines-hidden"
)

string(CONCAT CMAKE_C_FLAGS_INIT
  "-mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar"
  " -Os -g -fomit-frame-pointer -fno-builtin -Wall -Werror"
  " -fno-asynchronous-unwind-tables -fvisibility=hidden -fvisibility-inlines-hidden"
)

set(CMAKE_ASM_FLAGS_INIT "-mcpu=cheriot -mabi=cheriot -mxcheri-rvc -mrelax -fshort-wchar")

set(CMAKE_EXE_LINKER_FLAGS_INIT "-nodefaultlibs -fuse-ld=lld -T \"${LINKER_SCRIPT}\"")
