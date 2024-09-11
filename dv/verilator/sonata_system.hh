// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "verilated_toplevel.h"
#include "verilator_memutil.h"

class SonataSystem {
 public:
  SonataSystem(const char *ram_hier_path, int ram_size_words,
    const char *hyperram_hier_path, int hyperram_size_words);
  virtual ~SonataSystem() {}
  virtual int Main(int argc, char **argv);


 protected:
  top_verilator _top;
  VerilatorMemUtil _memutil;
  MemArea _ram;
  MemArea _hyperram;

  virtual int Setup(int argc, char **argv, bool &exit_app);
  virtual void Run();
  virtual bool Finish();
};
