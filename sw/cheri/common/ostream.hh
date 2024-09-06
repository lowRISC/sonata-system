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

  class OStream {
    public: 
      enum class Radix { Bin=2, Oct=8, Dec=10, Hex=16};

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
  };
}
