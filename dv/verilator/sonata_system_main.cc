// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sonata_system.h"

int main(int argc, char **argv) {
  SonataSystem sonata_system(
      "TOP.top_verilator.u_sonata_system.u_sram_top.u_ram.gen_generic.u_impl_generic",
      1024 * 1024);

  return sonata_system.Main(argc, argv);
}
