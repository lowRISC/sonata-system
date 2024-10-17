# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  pythonEnv,
  sonata-system-software,
  sonataGatewareFiles,
  FLAKE_GIT_COMMIT,
  FLAKE_GIT_DIRTY,
}: let
  inherit (pkgs.lib) fileset;

  fpgaFiles = fileset.unions [
    sonataGatewareFiles
    (fileset.fileFilter (file: file.hasExt "xdc") ../data)
    ../flow
  ];

  fpgaFileset = fileset.toSource {
    root = ../.;
    fileset = fpgaFiles;
  };

  bootloaderDependancies =
    fileset.difference
    (fileset.unions [
      ../sw/cheri
      ../sw/common
    ])
    (fileset.unions [
      ../sw/cheri/tests
      ../sw/cheri/checks
      ../sw/cheri/error_leds
      ../sw/cheri/sim_boot_stub
      ../sw/cheri/README.md
    ]);
in {
  # The only files we expect the fpga build depends.
  fpgaDependanciesFileset = fileset.toSource {
    root = ../.;
    fileset = fileset.unions [
      fpgaFiles
      bootloaderDependancies
    ];
  };

  bitstream-build = pkgs.writeShellApplication {
    name = "bitstream-build";
    runtimeInputs = [pythonEnv];
    runtimeEnv = {inherit FLAKE_GIT_COMMIT FLAKE_GIT_DIRTY;};
    text = ''
      fusesoc --cores-root=${fpgaFileset} \
        run --target=synth --build lowrisc:sonata:system \
        --SRAMInitFile=${sonata-system-software}/share/boot_loader.vmem
    '';
  };

  bitstream-load = pkgs.writeShellApplication {
    name = "bitstream-load";
    runtimeInputs = [pythonEnv pkgs.openfpgaloader];
    text = ''
      BITSTREAM=$(find ./ -type f -name "lowrisc_sonata_system_0.bit")
      openFPGALoader -c ft4232 "$BITSTREAM"
    '';
  };
}
