// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sonata_system.h"

int main(int argc, char **argv) {
  SonataSystem sonata_system(
      "TOP.top_verilator.u_sonata_system.u_sram_top.u_ram.gen_generic.u_impl_generic",
      32 * 1024, // 32k words = 128 KiB
      "TOP.top_verilator.u_sonata_system.g_hyperram.u_hyperram.u_hyperram_model.u_ram.gen_generic.u_impl_generic",
      256 * 1024 // 256k words = 1 MiB
  );

  return sonata_system.Main(argc, argv);
}
