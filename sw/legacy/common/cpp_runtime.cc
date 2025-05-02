// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <cstdlib>

void* operator new(size_t size) { return malloc(size); }

void* operator new[](size_t size) { return operator new(size); }

void operator delete(void* ptr) noexcept { free(ptr); }

void operator delete[](void* ptr) noexcept { operator delete(ptr); }

void operator delete(void* ptr, size_t) noexcept { operator delete(ptr); }

void operator delete[](void* ptr, size_t) noexcept { operator delete[](ptr); }
