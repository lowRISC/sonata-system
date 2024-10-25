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
    rev = "c29bdeccc0335b81acea09c069fc94ca044a5a14";
    fetchSubmodules = true;
    hash = "sha256-44xABnopOfF8TilGQNVTynfRc0cUiiAk7E5KoRZ0Hw0=";
  };

  reisfmtSource = pkgs.fetchFromGitHub {
    owner = "engdoreis";
    repo = "reisfmt";
    rev = "38d3311f01c14bd184278b8fbf49d096367cd479";
    fetchSubmodules = true;
    hash = "sha256-uemJ/o8ktcrufos172pcQEzVbH4B62Nkgg+hnEwGvak=";
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
