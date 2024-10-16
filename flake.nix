# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  description = "Sonata System";
  inputs = {
    lowrisc-nix.url = "github:lowRISC/lowrisc-nix";

    nixpkgs.follows = "lowrisc-nix/nixpkgs";
    flake-utils.follows = "lowrisc-nix/flake-utils";
    poetry2nix.follows = "lowrisc-nix/poetry2nix";
  };

  nixConfig = {
    extra-substituters = ["https://nix-cache.lowrisc.org/public/"];
    extra-trusted-public-keys = ["nix-cache.lowrisc.org-public-1:O6JLD0yXzaJDPiQW1meVu32JIDViuaPtGDfjlOopU7o="];
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    lowrisc-nix,
    lowrisc-it,
    ...
  } @ inputs: let
    system_outputs = system: let
      FLAKE_GIT_COMMIT =
        if (self ? rev)
        then self.rev
        else self.dirtyRev;
      FLAKE_GIT_DIRTY =
        if (self ? rev)
        then false
        else true;

      pkgs = import nixpkgs {
        inherit system;
      };

      lrDoc = lowrisc-nix.lib.doc {inherit pkgs;};
      lrPkgs = lowrisc-nix.outputs.packages.${system};
      inherit (pkgs.lib) fileset getExe;

      pythonEnv = let
        poetry2nix = inputs.poetry2nix.lib.mkPoetry2Nix {inherit pkgs;};
        poetryOverrides = lowrisc-nix.lib.poetryOverrides {inherit pkgs;};
      in
        poetry2nix.mkPoetryEnv {
          projectDir = ./.;
          python = pkgs.python310;
          overrides = [
            poetryOverrides
            poetry2nix.defaultPoetryOverrides
          ];
        };

      sonata-documentation = lrDoc.buildMdbookSite {
        version = "";
        pname = "sonata-documentation";
        src = fileset.toSource {
          root = ./.;
          fileset = fileset.unions [
            (lrDoc.standardMdbookFileset ./.)
            ./util/mdbook
            ./util/mdbook_wavejson.py
          ];
        };
      };

      cheriotPkgs = lowrisc-nix.outputs.devShells.${system}.cheriot.nativeBuildInputs;

      sonataGatewareFiles = fileset.unions [
        ./rtl
        ./vendor
        ./sonata.core
        ./sonata_system.core
        ./rv_timer.core
        ./pulp_riscv_dbg.core
        ./open_hbmc.core
      ];

      sonataSimulatorFileset = fileset.toSource {
        root = ./.;
        fileset = fileset.unions [
          sonataGatewareFiles
          ./dv
        ];
      };

      sonata-simulator = pkgs.stdenv.mkDerivation rec {
        name = "sonata-simulator";
        src = sonataSimulatorFileset;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.verilator pythonEnv];
        inherit FLAKE_GIT_COMMIT;
        inherit FLAKE_GIT_DIRTY;
        buildPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=sim --setup --build lowrisc:sonata:system \
            --verilator_options="-j $NIX_BUILD_CORES" --make_options="-j $NIX_BUILD_CORES"
        '';
        installPhase = ''
          mkdir -p $out/bin/
          cp -r build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator $out/bin/${name}
        '';
        meta.mainProgram = name;
      };

      sonata-sim-boot-stub = pkgs.stdenv.mkDerivation {
        name = "sonata-sim-boot-stub";
        src = sw/cheri/sim_boot_stub/.;
        nativeBuildInputs = cheriotPkgs;
        buildPhase = ''
          make
        '';
        installPhase = ''
          mkdir -p $out/share/
          cp -r sim_boot_stub $out/share
        '';
      };

      software = import nix/software.nix {
        inherit pkgs lrPkgs cheriotPkgs;
      };

      bitstream = import nix/bitstream.nix {
        inherit
          pkgs
          pythonEnv
          sonataGatewareFiles
          FLAKE_GIT_COMMIT
          FLAKE_GIT_DIRTY
          ;
        inherit (software) sonata-system-software;
      };

      tests = import nix/tests.nix {
        inherit
          pkgs
          pythonEnv
          sonata-sim-boot-stub
          sonata-simulator
          ;
        inherit (software) sonata-system-software cheriot-rtos-test-suite;
      };

      lint = import nix/lint.nix {
        inherit
          pkgs
          pythonEnv
          sonataSimulatorFileset
          FLAKE_GIT_COMMIT
          FLAKE_GIT_DIRTY
          ;
      };
    in {
      formatter = pkgs.alejandra;
      devShells.default = pkgs.mkShell {
        name = "sonata-system-devshell";
        packages =
          (with pkgs; [
            cmake
            screen
            picocom
            srecord
            gtkwave
            openfpgaloader
            openocd
          ])
          ++ (with lrPkgs; [
            uf2conv
            # For legacy software
            lowrisc-toolchain-gcc-rv32imcb
          ])
          ++ (with sonata-simulator; buildInputs ++ nativeBuildInputs);
      };
      packages = {
        inherit
          sonata-simulator
          sonata-sim-boot-stub
          sonata-documentation
          ;
        inherit (software) sonata-system-software cheriot-rtos-test-suite;
        test-simulator = tests.simulator;
        sonata-simulator-lint = lint.sonata-simulator;
        bitstream-build = bitstream.build;
        bitstream-load = bitstream.load;
      };
      apps = builtins.listToAttrs (map (program: {
        inherit (program) name;
        value = {
          type = "app";
          program = getExe program;
        };
      }) [lint.all lint.python tests.fpga lint.cpp]);
    };
  in
    flake-utils.lib.eachDefaultSystem system_outputs;
}
