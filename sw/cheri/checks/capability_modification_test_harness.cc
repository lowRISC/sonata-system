/**
  * Copyright lowRISC contributors.
  * Licensed under the Apache License, Version 2.0, see LICENSE for details.
  * SPDX-License-Identifier: Apache-2.0
  */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>
#include <ds/xoroshiro.h>

#include <stdint.h>
#include <stdbool.h>
#include <array>

#include <platform-timer.hh>
#include <platform-uart.hh>

#include "../common/sonata-devices.hh"
#include "../common/flash-utils.hh"
#include "../common/uart-utils.hh"
#include "../common/console.hh"

using namespace CHERI;

void write_hex32b(UartPtr uart, uint32_t value)
{
    // Print highest byte first (big endian style)
    write_hex8b(uart, (value >> 24) & 0xFF);
    write_hex8b(uart, (value >> 16) & 0xFF);
    write_hex8b(uart, (value >> 8)  & 0xFF);
    write_hex8b(uart, (value >> 0)  & 0xFF);
}

int print_capability(Capability<void> ptr, UartPtr uart)
{
    volatile uint32_t addr = ptr.address();
    volatile uint32_t bounds = ptr.bounds();
    volatile uint32_t base = ptr.base();
    volatile uint32_t top = ptr.top();
    PermissionSet perms = ptr.permissions();
    bool perm_global = perms.contains(Permission::Global);
    bool perm_load_global = perms.contains(Permission::LoadGlobal);
    bool perm_store = perms.contains(Permission::Store);
    bool perm_load_mutable = perms.contains(Permission::LoadMutable);
    bool perm_store_local = perms.contains(Permission::StoreLocal);
    bool perm_load = perms.contains(Permission::Load);
    bool perm_load_store_capability = perms.contains(Permission::LoadStoreCapability);
    bool perm_access_system_registers = perms.contains(Permission::AccessSystemRegisters);
    bool perm_execute = perms.contains(Permission::Execute);
    bool perm_unseal = perms.contains(Permission::Unseal);
    bool perm_seal = perms.contains(Permission::Seal);
    bool perm_user0 = perms.contains(Permission::User0);


    uint32_t tag = ptr.is_valid();
    uint32_t seal = ptr.type();
    write_str(uart, "Printing capability...\n");
    write_str(uart, "    Base: ");
    write_hex32b(uart, base);
    write_str(uart, "\n    Top:  ");
    write_hex32b(uart, top);
    write_str(uart , "\n    Address: ");
    write_hex32b(uart, addr);
    write_str(uart , "\n    Length:");
    write_hex32b(uart, bounds);
    write_str(uart , "\n    Tag: ");
    write_hex8b(uart, tag);
    write_str(uart , "\n    OType: ");
    write_hex8b(uart, seal);
    write_str(uart , "\n");
    if (perm_execute){
        write_str(uart, "EX ");
    }
    if (perm_access_system_registers){
        write_str(uart, "SR ");
    }
    if (perm_seal){
        write_str(uart, "SE ");
    }
    if (perm_unseal){
        write_str(uart, "US ");
    }
    if (perm_user0){
        write_str(uart, "U0 ");
    }
    if (perm_global){
        write_str(uart, "GL ");
    }
    if (perm_store_local){
        write_str(uart, "SL ");
    }
    if (perm_load_mutable){
        write_str(uart, "LM ");
    }
    if (perm_load_global){
        write_str(uart, "LG ");
    }
    if (perm_load_store_capability){
        write_str(uart, "MC ");
    }
    if (perm_store){
        write_str(uart, "SD ");
    }
    if (perm_load){
        write_str(uart, "LD ");
    }
    write_str(uart, "\n");




    return 1;
}

// Testing the CIncAddr instruction
Capability<void> CIncAddr_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_val = prng();
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    asm volatile(
      "cmove ca0, %[cap]\n"      // ca0 = cap
      "mv a2, %[rand]\n"         // a2 = rand
      "cgettop a3, ca0\n"        // a3 = ca0.top()
      "cgetaddr a4, ca0\n"       // a4 = ca0.addr()
      "sub a3, a3, a4\n"         // a4 = ca0.top() - ca0.addr()
      "beqz a3, 1f\n"            // if a3 == 0 goto 1
      "remu a2, a2, a3\n"
      "cincoffset ca0, ca0, a2\n"
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap)
      : [cap] "C"(right_type_cap), [rand] "r"(rand_val)
      : "ca0", "a2", "a3"
    );
    out = right_type_cap;
    bool failed = !out.is_valid();
    return out;
}

// TODO: figure out a way of doing this test
// Testing the CIncAddrImm instruction
int CIncAddrImm_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t imm = prng() % 0b1000000000000;
    // Not sure how to do this because the immediate must be set at compile time which I can't do in this framework
    // Despite this method should be pretty much the same
}

// Testing the CSetBounds instruction
Capability<void> CSetBounds_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_base = prng();
    uint32_t rand_length = prng();
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    uint32_t test_val;
    asm volatile(
      "cmove ca0, %[cap]\n"        // ca0 = cap
      "mv a2, %[base]\n"           // a2 = rand_base
      "mv a3, %[length]\n"         // a3 = rand_length
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.top() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a2, a2, a4\n"          // a2 = a2 % a4
      "cincoffset ca0, ca0, a2\n"  // ca0.addr += rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.base() + ca0.bounds() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a3, a3, a4\n"          //
      "csetbounds ca0, ca0, a3\n"  // ca0.bounds = rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap), [test_v] "=r"(test_val)
      : [cap] "C"(right_type_cap), [base] "r"(rand_base), [length] "r"(rand_length)
      : "ca0", "a2", "a3", "a4", "a5"
    );
    out = right_type_cap;
    bool failed = !out.is_valid();
    return out;
}

// Testing the CSetBoundsExact instruction
Capability<void> CSetBoundsExact_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_base = prng();
    uint32_t rand_length = prng();
    uint32_t leading_zeroes_rand_base = __builtin_ctz(cap.address() + rand_base % (cap.top() - cap.address()));
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    uint32_t test_val;
    asm volatile(
      "cmove ca0, %[cap]\n"        // ca0 = cap
      "mv a2, %[base]\n"           // a2 = rand_base
      "mv a3, %[length]\n"         // a3 = rand_length
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.top() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a2, a2, a4\n"          // a2 = a2 % a4
      "cincoffset ca0, ca0, a2\n"  // ca0.addr += rand % (ca0.top - ca0.addr)
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.base() + ca0.bounds() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a3, a3, a4\n"          //
      "mv a2, %[clz]\n"            // a4 = clz ca0.addr + rand % (ca0.top - ca0.addr)
//      "li t0, 23\n"
//      "sub a2, t0, a2\n "         // a2 = e
//      "li  t0, 15\n"              // t0 = 15
//      "li  t1, 24\n"              // t1 = 24
//      "bleu a2, t0, 1f\n"         // If a2 <= 15, skip setting
//      "mv  a2, t1\n"              //a2 = 24
//      "1:\n"
      "mv %[test_v], a2\n"
      "li t0, 1\n"                 // t0 = 1
      "sll t0, t0, a2\n"           // t0 = 1 << a2
      "addi t0, t0, -1\n"          // t0 = (1 << a2) - 1
      "not t0, t0\n"
      "and a3, a3, t0\n"
      "mv %[test_v], a3\n"

      "li t0, 1\n"                 // t0 = 1
      "sll t0, t0, a2\n"           // t0 = 1 << a2
      "slli t0, t0, 9\n"
      "addi t0, t0, -1\n"          // t0 = (1 << a2) - 1
      "and a3, a3, t0\n"

      "csetboundsexact ca0, ca0, a3\n"  // ca0.bounds = rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap), [test_v] "=r"(test_val)
      : [cap] "C"(right_type_cap), [base] "r"(rand_base), [length] "r"(rand_length), [clz] "r"(leading_zeroes_rand_base)
      : "ca0", "a2", "a3", "a4", "a5", "t0", "t1"
    );
//    write_str(uart, "e: ");
//    write_hex32b(uart, test_val);
//    write_str(uart, "\n");
//
//    write_str(uart, "Number: ");
//    write_hex32b(uart, rand_base);
//    write_str(uart, "\n");

    out = right_type_cap;
    bool failed = !out.is_valid();
    return out;
}

// Testing the CSetBoundsRoundDown instruction
Capability<void> CSetBoundsRoundDown_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_base = prng();
    uint32_t rand_length = prng();
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    uint32_t test_val;
    asm volatile(
      "cmove ca0, %[cap]\n"        // ca0 = cap
      "mv a2, %[base]\n"           // a2 = rand_base
      "mv a3, %[length]\n"         // a3 = rand_length
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.top() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a2, a2, a4\n"          // a2 = a2 % a4
      "cincoffset ca0, ca0, a2\n"  // ca0.addr += rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "cgettop a4, ca0\n"          // a4 = ca0.top()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "sub a4, a4, a5\n"           // a4 = ca0.base() + ca0.bounds() - ca0.addr()
      "beqz a4, 1f\n"
      "remu a3, a3, a4\n"          //
      "csetboundsrounddown ca0, ca0, a3\n"  // ca0.bounds = rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap), [test_v] "=r"(test_val)
      : [cap] "C"(right_type_cap), [base] "r"(rand_base), [length] "r"(rand_length)
      : "ca0", "a2", "a3", "a4", "a5"
    );
    out = right_type_cap;
    bool failed = !out.is_valid();
    return out;
}

// Testing the CSetAddr instruction
Capability<void> CSetAddr_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_val = prng();
    uint32_t e;
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    asm volatile(
      "cmove ca0, %[cap]\n"      // ca0 = cap
      "mv a2, %[rand]\n"         // a2 = rand
      "cgetlen a3, ca0\n"        // a3 = ca0.length()
      "beqz a3, 1f\n"
      "remu a2, a2, a3\n"        // a2 = a2 % a4
      "cgetbase a3, ca0\n"       // a3 = ca0.base()
      "add a2, a2, a3\n"         // a2 = ca0.base() + ( rand % ca0.length() )
      "csetaddr ca0, ca0, a2\n"
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap)
      : [cap] "C"(right_type_cap), [rand] "r"(rand_val)
      : "ca0", "a2", "a3"
    );
    out = right_type_cap;
    return out;
}

Capability<void> CSeal_test(Capability<void> cap, Capability<void> seal_cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_val = prng();
    void* right_type_cap = cap.get();
    void* right_type_sealing_cap = seal_cap.get();
    Capability<void> out_sealed;
    Capability<void> out_sealer;
    uint32_t test_val;
    asm volatile(
      // Loading in the external values
      "cmove ca0, %[cap]\n"           // ca0 = cap
      "cmove ca1, %[s_cap]\n"         // ca1 = sealing_cap
      "mv a2, %[rand]\n"              // a2 = rand

      // Main body
      "cgetperm a3, ca0\n"           // a3 = ca0.perms

      "li a4, 0x100\n"               // a4 = 2^8

      "and a3, a3, a4\n"             // a3 = a3 & a4
      // If a3 is 0 then the executable permission is not set and if a3 is not zero then the executable permission is set

      "beqz a3, 1f\n"

      // The execute permission is set. The address of ca1 should be set between 1 and 7
      "li t0, 6\n"
      "remu a2, a2, t0\n"
      "addi a2, a2, 1\n"

      // Now check that the base is below 7 and the top is above 1
      "li t0, 8\n"
      "cgetbase t1, ca1\n"
      "bgeu t1, t0, 3f\n"

      "li t0, 1\n"
      "cgettop t1, ca1\n"
      "bgeu t0, t1, 3f\n"

      // This now needs to be constrained within the bounds of the capability
      "cgettop t0, ca1\n"
      "bgeu a2, t0, 4f\n"
      "j 5f\n"
      "4:\n"
      "mv a2, t0\n"
      "5:\n"

      "cgetbase t0, ca1\n"
      "bgeu t0, a2, 6f\n"
      "j 7f\n"
      "6:\n"
      "mv a2, t0\n"
      "7:\n"

      // The address can now be set
      "csetaddr ca1, ca1, a2\n"
      "j 2f\n"

      // The execute permission is not set
      "1:\n"
      "li t0, 7\n"
      "remu a2, a2, t0\n"
      "addi a2, a2, 9\n"

      // Now check that the base is below 15 and the top is above 9
      "li t0, 16\n"
      "cgetbase t1, ca1\n"
      "bgeu t1, t0, 3f\n"

      "li t0, 9\n"
      "cgettop t1, ca1\n"
      "bgeu t0, t1, 3f\n"

      // This now needs to be constrained within the bounds of the capability
      "cgettop t0, ca1\n"
      "bgeu a2, t0, 8f\n"
      "j 9f\n"
      "8:\n"
      "mv a2, t0\n"
      "9:\n"

      "cgetbase t0, ca1\n"
      "bgeu t0, a2, 10f\n"
      "j 11f\n"
      "10:\n"
      "mv a2, t0\n"
      "11:\n"
      "csetaddr ca1, ca1, a2\n"

      // Seal
      "2:\n"
      "li t0, 1\n"
      "mv %[test_v], t0\n"

      "cseal ca0, ca0, ca1\n"

      "3:\n"
      // Loading out the values
      "cmove %[out_cap], ca0\n"

      : [out_cap] "=C"(right_type_cap), [test_v] "=r"(test_val)
      : [cap] "C"(right_type_cap), [s_cap] "C"(right_type_sealing_cap), [rand] "r"(rand_val)
      : "ca0", "ca1", "ca3", "a2", "a3", "a4", "t0", "t1"
    );
//    write_str(uart, "Perms: ");
//    write_hex32b(uart, test_val);
//    write_str(uart, "\n");

    out_sealed = right_type_cap;
    return out_sealed;

}

Capability<void> CUnseal_test(Capability<void> cap, Capability<void> unseal_cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    void *right_type_cap = cap.get();
    void *right_type_unseal_cap = unseal_cap.get();
    uint32_t test_val;
    Capability<void> out;
    asm volatile(
      // Moving values into the registers
      "cmove ca0, %[cap]\n"          // ca0 = cap
      "cmove ca1, %[unseal_cap]\n"   // ca1 = unseal_cap

      // Main body

      // Check that the otype of cap is within the bounds on unseal_cap
      "cgettype a2, ca0\n"
      "cgetbase a3, ca1\n"
      // if ca0.type < ca1.base end
      "bgeu a3, a2, 1f\n"

      "cgettop a3, ca1\n"
      "bgeu a2, a3, 1f\n"

      // The otype of cap is within the bounds on unseal_cap

      "cunseal ca0, ca0, ca1\n"

      // Moving values out of the registers
      "1:\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap), [test_v] "=r"(test_val)
      : [cap] "C"(right_type_cap), [unseal_cap]"C"(right_type_unseal_cap)
      : "ca0", "a2", "a3"
    );
    write_str(uart, "Test value: ");
    write_hex32b(uart, test_val);
    write_str(uart, "\n");
    out = right_type_cap;
    return out;

}
Capability<void> randomise_capability(Capability<void> cap){
    void* right_type_cap = cap.get();
    return cap;
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot, void *sealRoot, void *exRoot) {
  Capability<void> root{rwRoot};
  Capability<void> seal_root{sealRoot};
  Capability<void> execute_root{exRoot};

  // Create a bounded capability to the UART
  UartPtr uart = uart_ptr(root, 0);
  uart->init(BAUD_RATE);
  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEE, 0xBAADCAFE);
  volatile uint32_t rand_val;
  write_str(uart, "\n\n\n\n\nRunning test...\n");
  Capability<void> cap = root.cast<void>();
  Capability<void> seal_cap = seal_root.cast<void>();
  Capability<void> ex_cap = execute_root.cast<void>();
  Capability<void> out_cap;
  int num_fails = 0;
  int num_passes = 0;

  for (int iterations = 0; iterations < 0x10; iterations++){
    cap = root.cast<void>();
    seal_cap = seal_root.cast<void>();
    ex_cap = execute_root.cast<void>();

//    print_capability(cap, uart);
//    print_capability(seal_cap, uart);
//    print_capability(ex_cap, uart);
    for (int i = 0; i <0x1; i++){
//       seal_cap = CSetBounds_test(seal_cap, prng, root, uart);
       seal_cap = CIncAddr_test(seal_cap, prng, root, uart);
       cap = CSeal_test(cap, seal_cap, prng, root, uart);
       out_cap = CUnseal_test(cap, seal_cap, prng, root, uart);
       //cap = CSetAddr_test(cap, prng, root, uart);
       if (!out_cap.is_valid()){
//          print_capability(out_cap, uart);
          write_str(uart, "Test failed\n");
          num_fails++;
          break;
       }
       print_capability(out_cap, uart);
    }
    if (out_cap.is_valid()){
        num_passes++;
    }

  }
  write_str(uart, "Number of failed tests: ");
  write_hex32b(uart, num_fails);
  write_str(uart, "\n");
  write_str(uart, "Number of passed tests: ");
  write_hex32b(uart, num_passes);
  write_str(uart, "\n");
  write_str(uart, "Tests completed.\n");
}