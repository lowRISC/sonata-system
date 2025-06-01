// Copyright (c) 2025 Douglas Reis.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/*
 * This file was copied over from zermio repository.
 */

#pragma once

#include <cstdint>
#include <limits>
namespace zermio {
enum Permissions : uint8_t { Read = 0x01, Write = 0x02, ReadWrite = 0x03 };

template <Permissions P>
concept Writable = (P & Permissions::Write) == Permissions::Write;
static_assert(Writable<Write> && Writable<ReadWrite> && !Writable<Read>);

template <Permissions P>
concept Readable = (P & Permissions::Read) == Permissions::Read;
static_assert(Readable<Read> && Readable<ReadWrite> && !Readable<Write>);

struct Register {
  const std::size_t addr = 0;
  std::size_t cache      = 0;

  inline void commit() { *(reinterpret_cast<volatile std::size_t*>(addr)) = cache; }
  inline void fetch() { cache = *(reinterpret_cast<volatile std::size_t*>(addr)); }
};

template <std::size_t OFFSET, std::size_t BITS, Permissions P>
class BitField {
  Register reg{0};

 public:
  static consteval std::size_t mask() {
    static_assert(BITS <= sizeof(std::size_t) * 8);
    if constexpr (BITS == sizeof(std::size_t) * 8) {
      return std::numeric_limits<std::size_t>::max();
    } else {
      return ((0x01 << BITS) - 1) << OFFSET;
    }
  }

  static consteval std::size_t max() { return (1 << BITS) - 1; }

  inline constexpr auto& write(const std::size_t value) {
    static_assert(BITS > 1, ">> Error: This bitfield is multibit. Try using set or reset. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    clear();
    reg.cache |= ((value << OFFSET) & mask());
    return *this;
  }

  inline constexpr auto& set() {
    static_assert(BITS == 1, ">> Error: This bitfield isn't multibit. Try using write or clear. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    reg.cache |= (0x01 << OFFSET);
    return *this;
  }

  inline constexpr auto& reset() {
    static_assert(BITS == 1, ">> Error: This bitfield isn't multibit. Try using write or clear. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    clear();
    return *this;
  }

  inline constexpr auto& toggle() {
    static_assert(BITS == 1, ">> Error: This bitfield isn't multibit. Try using write or clear. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    reg.cache ^= (0x01 << OFFSET);
    return *this;
  }

  inline constexpr auto& bit_mask(std::size_t value, std::size_t offset) {
    static_assert(BITS > 1, ">> Error: This bitfield is multibit. Try using set or reset. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    reg.cache &= ~((0x01 << (OFFSET + offset)) & mask());
    reg.cache |= ((value << (OFFSET + offset)) & mask());
    return *this;
  }

  inline constexpr auto& bit(bool bit) {
    static_assert(BITS == 1, ">> Error: This bitfield is multibit. Try using write or clear. <<");
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    reg.cache |= (static_cast<std::size_t>(bit) << OFFSET);
    return *this;
  }

  inline constexpr bool is_set() {
    static_assert(BITS == 1, ">> Error: This bitfield is multibit. Try using write or clear. <<");
    static_assert(Readable<P>, ">> Error: This bitfield can't be read. <<");
    return (reg.cache & mask()) == mask();
  }

  inline constexpr std::size_t get() {
    static_assert(BITS > 1, ">> Error: This bitfield is multibit. Try using set or reset. <<");
    static_assert(Readable<P>, ">> Error: This bitfield can't be read. <<");
    return (reg.cache & mask()) >> OFFSET;
  }

  inline constexpr auto& clear() {
    static_assert(Writable<P>, ">> Error: This bitfield can't be write. <<");
    reg.cache &= ~mask();
    return *this;
  }

  inline void commit() { reg.commit(); }
};
};  // namespace zermio
