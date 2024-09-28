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
    ...
  } @ inputs: let
    system_outputs = system: let
      version = "0.0.1";

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
          overrides = [
            poetryOverrides
            poetry2nix.defaultPoetryOverrides
          ];
        };

      sonata-documentation = lrDoc.buildMdbookSite {
        inherit version;
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

      lint-python = pkgs.writeShellApplication {
        name = "lint-python";
        runtimeInputs = [pythonEnv];
        text = ''
          ruff format --check .
          ruff check .
          mypy .
        '';
      };

      lint-all = pkgs.writers.writeBashBin "lint-all" ''
        set -e
        ${getExe pkgs.reuse} --suppress-deprecation lint
        ${getExe pkgs.lychee} --offline --no-progress .
        ${getExe lint-python}
        ${getExe lint-cpp}
      '';

      lint-cpp = pkgs.writeShellApplication {
        name = "lint-cpp";
        runtimeInputs = [pkgs.clang-tools_18];
        text = ''
          set +u
          EXCLUDE="sw/cheri/build"
          FILES=$(find sw -type f -path "$EXCLUDE" -prune -o \( -name "*.c" -o -name "*.cc" -o -name "*.h" -o -name "*.hh" \))
          ARG="$1"
          [ -z "$1" ] && ARG="check"
          case "$ARG" in
            check)
              echo "$FILES" | xargs clang-format -n --Werror
            ;;
            fix)
              echo "$FILES" | xargs clang-format -i
            ;;
          esac
        '';
      };

      cheriotPkgs = lowrisc-nix.outputs.devShells.${system}.cheriot.nativeBuildInputs;

      sonataSimulatorFileset = fileset.toSource {
        root = ./.;
        fileset = fileset.unions [
          ./rtl
          ./dv
          ./vendor
          ./sonata.core
          ./sonata_system.core
          ./rv_timer.core
          ./pulp_riscv_dbg.core
          ./open_hbmc.core
        ];
      };
      sonata-simulator-lint = pkgs.stdenvNoCC.mkDerivation {
        name = "sonta-simulator-lint";
        src = sonataSimulatorFileset;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.verilator pythonEnv];
        dontBuild = true;
        doCheck = true;
        checkPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=lint --setup --build lowrisc:sonata:system \
            --verilator_options="+define+RVFI -j $NIX_BUILD_CORES"
        '';
        installPhase = "mkdir $out";
      };
      sonata-simulator = pkgs.stdenv.mkDerivation rec {
        inherit version;
        pname = "sonata-simulator";
        src = sonataSimulatorFileset;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.verilator pythonEnv];
        buildPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=sim --setup --build lowrisc:sonata:system \
            --verilator_options="-j $NIX_BUILD_CORES" --make_options="-j $NIX_BUILD_CORES"
        '';
        installPhase = ''
          mkdir -p $out/bin/
          cp -r build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator $out/bin/${pname}
        '';
        meta.mainProgram = pname;
      };

      sonata-sim-boot-stub = pkgs.stdenv.mkDerivation {
        inherit version;
        pname = "sonata-sim-boot-stub";
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

      cheriotRtosSource = pkgs.fetchFromGitHub {
        owner = "lowRISC";
        repo = "CHERIoT-RTOS";
        rev = "444e4965c1dc84414e4959d55e47b0e999c086c9";
        fetchSubmodules = true;
        hash = "sha256-tXTIKmf/MBXmspcPdKYqQQp8zj6HuUgGXlj+uUCcC40=";
      };

      sonata-system-software = pkgs.stdenv.mkDerivation rec {
        inherit version;
        pname = "sonata-system-software";
        src = fileset.toSource {
          root = ./sw;
          fileset = fileset.unions [
            ./sw/cheri
            ./sw/common
          ];
        };
        sourceRoot = "${src.name}/cheri";
        nativeBuildInputs = cheriotPkgs ++ (with pkgs; [cmake]);
        cmakeFlags = [
          "-DFETCHCONTENT_SOURCE_DIR_CHERIOT_RTOS=${cheriotRtosSource}"
        ];
        dontFixup = true;
      };

      cheriot-rtos-test-suite = pkgs.stdenvNoCC.mkDerivation {
        name = "cheriot-rtos-test-suite";
        src = cheriotRtosSource;
        buildInputs = cheriotPkgs ++ [lrPkgs.uf2conv];
        dontFixup = true;
        buildPhase = ''
          xmake config -P ./tests/ --board=sonata-prerelease
          xmake -P ./tests/
          ${./util/elf-to-uf2.sh} ./build/cheriot/cheriot/release/test-suite
        '';
        installPhase = ''
          mkdir -p $out/share/
          cp build/cheriot/cheriot/release/* $out/share/
        '';
        meta = {
          description = "The CHERIoT RTOS test suite.";
          homepage = "https://github.com/CHERIoT-Platform/cheriot-rtos/tree/main/tests";
          license = pkgs.lib.licenses.mit;
        };
      };

      tests-fpga = pkgs.writeShellApplication {
        name = "tests-fpga";
        runtimeInputs = [pythonEnv];
        text = ''
          set +u
          if [ -z "$1" ]; then
            echo "Please provide the tty device location (e.g. /dev/ttyUSB2)" \
              "as the first argument."
            exit 2
          fi
          ${./util/test_runner.py} -t 30 fpga "$1" \
            --elf-file ${sonata-system-software}/bin/test_runner \
            --tcl-file ${./util/sonata-openocd-cfg.tcl}
          ${./util/test_runner.py} -t 600 fpga "$1" \
            --uf2-file ${cheriot-rtos-test-suite}/share/test-suite.uf2
        '';
      };

      tests-simulator = pkgs.stdenvNoCC.mkDerivation {
        name = "tests-simulator";
        src = ./.;
        dontBuild = true;
        doCheck = true;
        buildInputs = [sonata-simulator pythonEnv];
        checkPhase = ''
          python ${./util/test_runner.py} -t 60 sim \
              --elf-file ${sonata-system-software}/bin/test_runner
          python ${./util/test_runner.py} -t 600 sim \
              --sim-boot-stub ${sonata-sim-boot-stub.out}/share/sim_boot_stub \
              --elf-file ${cheriot-rtos-test-suite}/share/test-suite
        '';
        installPhase = "mkdir $out";
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
          sonata-simulator-lint
          sonata-sim-boot-stub
          sonata-documentation
          sonata-system-software
          cheriot-rtos-test-suite
          ;
      };
      checks = {inherit tests-simulator;};
      apps = builtins.listToAttrs (map (program: {
        inherit (program) name;
        value = {
          type = "app";
          program = getExe program;
        };
      }) [lint-all lint-python tests-fpga lint-cpp]);
    };
  in
    flake-utils.lib.eachDefaultSystem system_outputs;
}
