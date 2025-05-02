# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  lrPkgs,
  cheriotPkgs,
}: let
  inherit (pkgs.lib) fileset;

  cheriotRtosSource = pkgs.fetchFromGitHub {
    owner = "CHERIoT-Platform";
    repo = "CHERIoT-RTOS";
    rev = "e34c07efeb1919df6ee57d40c0f23fe8a22921b3";
    fetchSubmodules = true;
    hash = "sha256-36+fnc7hn7i6+jWeaBuFvrG0A7K+NcF4eaYpztolHes=";
  };

  reisfmtSource = pkgs.fetchFromGitHub {
    owner = "engdoreis";
    repo = "reisfmt";
    rev = "4ce04e1bc88d37ad359da051b91b4071f740c3d8";
    fetchSubmodules = true;
    hash = "sha256-sj4nyevLiCSEQk5KW8wea0rgAnI2quOi1h71dU+QYpU=";
  };

  lwipSource = pkgs.fetchFromGitHub {
    owner = "lwip-tcpip";
    repo = "lwip";
    rev = "0a0452b2c39bdd91e252aef045c115f88f6ca773";
    fetchSubmodules = true;
    hash = "sha256-92KHmCMrE3LiQ1Q03Tc51B2i3oCYhfvnYu4CiCJxVWU=";
  };
in {
  sonata-system-software = pkgs.stdenv.mkDerivation rec {
    name = "sonata-system-software";
    src = fileset.toSource {
      root = ../.;
      fileset = fileset.unions [
        ../sw/cheri
        ../sw/common
        ../vendor/display_drivers/st7735
      ];
    };
    sourceRoot = "${src.name}/sw/cheri";
    nativeBuildInputs = cheriotPkgs ++ (with pkgs; [cmake]);
    cmakeFlags = [
      "-DFETCHCONTENT_SOURCE_DIR_CHERIOT_RTOS=${cheriotRtosSource}"
      "-DFETCHCONTENT_SOURCE_DIR_REISFMT=${reisfmtSource}"
    ];
    dontFixup = true;
  };
  sonata-system-legacy-software = pkgs.stdenv.mkDerivation rec {
    name = "sonata-system-legacy-software";
    src = fileset.toSource {
      root = ../.;
      fileset = fileset.unions [
        ../sw/legacy
        ../sw/legacy/common
        ../vendor/display_drivers
        ../vendor/lowrisc_ibex/vendor/eembc_coremark
        ../vendor/lowrisc_ibex/examples/sw/benchmarks/coremark/ibex
        ../vendor/cheriot_debug_module/tb/prog
      ];
    };
    postPatch = ''
      cp -r --no-preserve=mode ${lwipSource} /tmp/lwip
    '';
    sourceRoot = "${src.name}/sw/legacy";
    nativeBuildInputs = (with pkgs; [cmake srecord]) ++ (with lrPkgs; [lowrisc-toolchain-gcc-rv32imcb]);
    cmakeFlags = [
      "-DFETCHCONTENT_SOURCE_DIR_LWIP=/tmp/lwip"
      "-DFETCHCONTENT_SOURCE_DIR_REISFMT=${reisfmtSource}"
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
      ${../util/elf-to-uf2.sh} ./build/cheriot/cheriot/release/test-suite
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
}
