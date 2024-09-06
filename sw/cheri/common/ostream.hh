/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART
#include <cheri.hh>
#include "../../common/defs.h"
#include <platform-uart.hh>
#include "sonata-peripherals.hh"
#include <stdlib.h>

namespace LOG {
  constexpr char const* endl = "\r\n";

  constexpr char const *consoleBold = "1";
  constexpr char const * consoleRed = "31";
  constexpr char const * consoleGreen = "32";
  constexpr char const * consoleReset = "0";

  enum class Radix { Bin=2, Oct=8, Dec=10, Hex=16};

  template<size_t N, typename T> requires std::integral<T>
    static inline char * to_hex_str(std::array<char, N> &buf,  T num) {
      assert(N > sizeof(T) * 2 + 1);
      constexpr T shift  = (sizeof(T) * 8 - 4);

      size_t i = 0;
      for (; i < (sizeof(T) * 2); ++i) {
        if (((num >> shift)& 0xf) < 10) {
          buf[i] = ((num >> shift) & 0xf) + '0';
        } else {
          buf[i] = ((num >> shift) & 0xf) + 'a' - 10;
        }
        num <<= 4;
      }
      buf[i] = 0;
      return buf.data();
    }

  template<size_t N, typename T> requires std::integral<T>
    static inline char * to_str(std::array<char, N> &buf,  T num) {
      size_t head = 0;
      size_t tail = N - 1;
      if constexpr (std::signed_integral<T>) {
        // This code won't be linked for unsigned T.
        if (num < 0) {
          buf[head++] = '-';
          num *= -1;
        }
      }

      for (; tail > 0 && num > 0; --tail) {
        buf[tail] = num % 10 + '0' ;
        num /= 10;
      }

      tail++;
      size_t len = N - tail + head;
      for (size_t i = 0; i < len; ++i) {
        buf[head+i] = buf[tail + i] ;
      }
      buf[len] = 0;
      return buf.data();
    }

  class OStream {
    protected:
      UartPtr fd;
      Radix radix;
      std::array<char, 32> buffer;

    public:
      OStream(UartPtr fd): fd(fd), radix(Radix::Dec){};

      inline OStream& operator<<(const char* str){
        for (; *str != '\0'; ++str) {
          fd->blocking_write(*str);
        }
        return *this;
      }

      inline OStream& operator<<(const Radix radix){
        this->radix=radix;
        return *this;
      }

      template<typename T> requires std::integral<T>
        inline OStream& operator<<(const T value){
          switch (radix){
            case Radix::Dec: 
              *this << to_str(buffer, value);
              break;
            case Radix::Hex: 
              *this << "0x" << to_hex_str(buffer, value);
              break;
            default:
              *this << "LOG::Ostream: Format not supported" ;
          }

          return *this;
        }
  };

  [[maybe_unused]] static inline void set_console_mode(OStream& console, const char *cc) {
    console << "\x1b[" << cc << "m";
  }
}
