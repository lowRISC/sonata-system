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
    owner = "lowRISC";
    repo = "CHERIoT-RTOS";
    rev = "bcd25c8734edb5395ac239b16af336987da45e9d";
    fetchSubmodules = true;
    hash = "sha256-UzvJyyavnWwg7XoF1C5LbVaFRm5Ib3V7Sy3Tl/Rmomg=";
  };

  reisfmtSource = pkgs.fetchFromGitHub {
    owner = "engdoreis";
    repo = "reisfmt";
    rev = "4ce04e1bc88d37ad359da051b91b4071f740c3d8";
    fetchSubmodules = true;
    hash = "sha256-sj4nyevLiCSEQk5KW8wea0rgAnI2quOi1h71dU+QYpU=";
  };
in {
  sonata-system-software = pkgs.stdenv.mkDerivation rec {
    name = "sonata-system-software";
    src = fileset.toSource {
      root = ../sw;
      fileset = fileset.unions [
        ../sw/cheri
        ../sw/common
      ];
    };
    sourceRoot = "${src.name}/cheri";
    nativeBuildInputs = cheriotPkgs ++ (with pkgs; [cmake]);
    cmakeFlags = [
      "-DFETCHCONTENT_SOURCE_DIR_CHERIOT_RTOS=${cheriotRtosSource}"
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
