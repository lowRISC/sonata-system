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
      "mv %[bigsum], a4\n"
      : [out_cap] "=C"(right_type_cap), [bigsum] "=r"(sum)
      : [cap] "C"(right_type_cap), [base] "r"(rand_base), [length] "r"(rand_length)
      : "ca0", "a2", "a3", "a4", "a5"
    );
    out = right_type_cap;
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
  prng.set_state(0xDEADBEE7, 0xBAADCAFE);
  volatile uint32_t rand_val;
  write_str(uart, "\n\n\n\n\nRunning test...\n");
  Capability<void> cap = root.cast<void>();
  Capability<void> cap = root.cast<void>();
  int num_fails = 0;
  for (int iterations = 0; iterations <10; iterations++){
    for (int i = 0; i <100; i++){
       cap = CSetBounds_test(cap, prng, root, uart);
       cap = CIncAddr_test(cap, prng, root, uart);
       if (!cap.is_valid()){
          write_str(uart, "Test failed\n");
          num_fails++;
          break;
       }
    }
  }
  write_str(uart, "Number of failed tests: ");
  write_hex32b(uart, num_fails);
  write_str(uart, "\n");
  write_str(uart, "Test complete.\n");

}