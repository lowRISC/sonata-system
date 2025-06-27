// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sonata_system.hh"

int main(int argc, char **argv) {
  SonataSystem sonata_system(
      "TOP.top_verilator.u_sonata_system.u_sram_top.u_ram.gen_generic.u_impl_generic",
      32 * 1024, // 32k words = 128 KiB
#ifdef USE_HYPERRAM_SRAM_MODEL
      // Simple SRAM model used within the Sonata System for faster simulations.
      "TOP.top_verilator.u_sonata_system.u_hyperram.gen_dual_port.u_hyperram_model.u_ram.gen_generic.u_impl_generic",
#else
      // HyperRAM simulation model external to the Sonata System; driven by HBMC.
      "TOP.top_verilator.u_hyperram_W956.u_ram.gen_generic.u_impl_generic",
#endif
      256 * 1024 // 256k words = 1 MiB
  );

  return sonata_system.Main(argc, argv);
}
