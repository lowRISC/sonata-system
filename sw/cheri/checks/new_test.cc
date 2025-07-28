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

Capability<void> randomise_capability(Capability<void> cap){
    void* right_type_cap = cap.get();
    return cap;
}

int print_capability(Capability<void> ptr, UartPtr uart)
{
    volatile uint32_t addr = ptr.address();
    volatile uint32_t bounds = ptr.bounds();
    volatile uint32_t base = ptr.base();
    volatile uint32_t top = ptr.top();
    uint32_t tag = ptr.is_valid();
    uint32_t seal = ptr.type();
    write_str(uart, "Printing capability...\n");
    write_str(uart, "    Base: ");
    write_hex32b(uart, base);
    write_str(uart, "\n    Top: ");
    write_hex32b(uart, top);
    write_str(uart , "\n    Address: ");
    write_hex32b(uart, addr);
    write_str(uart , "\n    Length:");
    write_hex32b(uart, bounds);
    write_str(uart , "\n    Tag: ");
    write_hex8b(uart, tag);
    write_str(uart , "\n    Seal: ");
    write_hex8b(uart, seal);
    write_str(uart , "\n");
    return 1;
}

// Testing the CIncAddr instruction
Capability<void> CIncAddr_test(Capability<void> cap, ds::xoroshiro::P64R32& prng, Capability<void> root, UartPtr uart){
    uint32_t rand_val = prng();
    write_str(uart, "Random value: ");
    write_hex32b(uart, rand_val);
    write_str(uart, "\n");
    void* right_type_cap = cap.get();
    Capability<void> out = root.cast<void>();
    asm volatile(
      "cmove ca0, %[cap]\n"      // ca0 = cap
      "mv a2, %[rand]\n"         // a2 = rand
      "cgetlen a3, ca0\n"        // a3 = ca0.bounds()
      "cgetaddr a4, ca0\n"       // a4 = ca0.addr()
      "cgetbase a5, ca0\n"       // a5 = ca0.base()
      "sub a4, a4, a5\n"         // a4 = ca0.addr() - ca0.base()
      "sub a3, a3, a4\n"           // a3 = ca0.bounds() - a4
      "remu a2, a2, a3\n"
      "cincoffset ca0, ca0, a2\n"
      "cmove %[out_cap], ca0\n"
      : [out_cap] "=C"(right_type_cap)
      : [cap] "C"(right_type_cap), [rand] "r"(rand_val)
      : "ca0", "a2", "a3"
    );
    out = right_type_cap;
    print_capability(out, uart);
    bool failed = !out.is_valid();
    return out;
}

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
    uint32_t sum;
    asm volatile(
      "cmove ca0, %[cap]\n"        // ca0 = cap
      "mv a2, %[base]\n"           // a2 = rand_base
      "mv a3, %[length]\n"         // a3 = rand_length
      "cgetlen a4, ca0\n"          // a4 = ca0.bounds()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "cgetbase t0, ca0\n"         // t0 = ca0.base()
      "sub t0, a5, t0\n"           // t0 = ca0.addr() - ca0.base()
      "sub a4, a4, t0\n"           // a4 = ca0.base() + ca0.bounds() - ca0.addr()
      "remu a2, a2, a4\n"          // a2 = a2 % a4
      "cincoffset ca0, ca0, a2\n"  // ca0.addr += rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "cgetlen a4, ca0\n"          // a4 = ca0.bounds()
      "cgetaddr a5, ca0\n"         // a5 = ca0.addr()
      "cgetbase t0, ca0\n"         // t0 = ca0.base()
      "sub t0, a5, t0\n"           // t0 = ca0.addr() - ca0.base()
      "sub a4, a4, t0\n"           // a4 = ca0.base() + ca0.bounds() - ca0.addr()
      "remu a3, a3, a4\n"          //
      "csetbounds ca0, ca0, a3\n"  // ca0.bounds = rand % (ca0.base() + ca0.bounds() - ca0.addr())
      "cmove %[out_cap], ca0\n"
      "mv %[bigsum], a4\n"
      : [out_cap] "=C"(right_type_cap), [bigsum] "=r"(sum)
      : [cap] "C"(right_type_cap), [base] "r"(rand_base), [length] "r"(rand_length)
      : "ca0", "a2", "a3", "a4", "a5", "t0"
    );
    write_hex32b(uart, sum);
    write_str(uart, "\n");
    out = right_type_cap;
    print_capability(out, uart);
    bool failed = !out.is_valid();
    return out;

}
/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  UartPtr uart = uart_ptr(root, 0);
  uart->init(BAUD_RATE);

  ds::xoroshiro::P64R32 prng;
  prng.set_state(0xDEADBEEF, 0xBAADCAFE);
  volatile uint32_t rand_val;
  write_str(uart, "\n\n\n\n\nRunning test...\n");
  Capability<void> cap = root.cast<void>();

  for (int i = 0; i <100; i++){
     cap = CSetBounds_test(cap, prng, root, uart);
     cap = CIncAddr_test(cap, prng, root, uart);
     if (!cap.is_valid()){
        write_str(uart, "Test failed\n");
        break;
     }

//      rand_val = prng();
//      uint32_t adjusted_rand_val;
////      write_hex32b(uart, rand_val);
////      write_str(uart, "\n");
//      Capability<void> cap = root.cast<void>();
//      Capability<void> out = root.cast<void>();
//      volatile uint32_t tagbit;
//      volatile uint32_t out_tagbit;
//      //asm("mv x0, %0" : : "r"(cap) : "x0");
//      //asm("ccleartag c%0, c%1" :"=r"(out) : "r"(cap));
//      void* right_type_cap = cap.get();
//      //volatile register uint32_t off asm("a0") = small_bounds;
//      asm volatile(
//        "cmove ca0, %[cap]\n"
//        "mv a3, %[off]\n"
//        "cgetlen a2, ca0\n"
//        "remu a2, a3, a2\n"
//        "cincoffset ca0, ca0, a2\n"
//        "cmove %[out_cap], ca0\n"
//        : [out_cap] "=C"(right_type_cap)
//        : [cap] "C"(right_type_cap), [off] "r"(rand_val), [adjusted_off_in] "r"(adjusted_rand_val)
//        : "ca0", "a2", "a3"
//      );
    //  asm("cmove ca0, %0" : : "C"(right_type_cap) : "ca0");
    //  asm("cgetbounds %0, ca0" : "=r"(top) : :);
    //  asm("ccleartag ca0, ca0" : : : "ca0");
    //  asm("cincoffsetimm ca0, ca0, 8" : : : "ca0");
    //  asm("cincoffset ca0, ca0, %0" : : "r"(rand_val) : "ca0");
    //  asm("cgetbase %0, ca0" : "=r"(base) : : "ca0");
    //  asm("cgettop %0, ca0" : "=r"(top) : : "ca0");
    //  asm("cmove %0, ca0" : "=C"(right_type_cap): : "ca0");
//      out = right_type_cap;
//      print_capability(out, uart);
  }
  write_str(uart, "Test complete.\n");

}