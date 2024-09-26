# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  lrPkgs,
  pythonEnv,
  FLAKE_GIT_COMMIT,
  FLAKE_GIT_DIRTY,
}: {
  build = pkgs.writeShellApplication {
    name = "bitstream-build";
    runtimeInputs = [pythonEnv lrPkgs.llvm_cheriot pkgs.cmake];
    runtimeEnv = {inherit FLAKE_GIT_COMMIT FLAKE_GIT_DIRTY;};
    text = ''
      cmake -B sw/cheri/build -S sw/cheri ;cmake --build sw/cheri/build
      fusesoc --cores-root=. run --target=synth --setup --build lowrisc:sonata:system
    '';
  };

  load = pkgs.writeShellApplication {
    name = "bitstream-load";
    runtimeInputs = [pythonEnv pkgs.openfpgaloader];
    text = ''
      BITSTREAM=$(find ./ -type f -name "lowrisc_sonata_system_0.bit")
      openFPGALoader -c ft4232 "$BITSTREAM"
    '';
  };
}
