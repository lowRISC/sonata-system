# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  pythonEnv,
  sonata-system-software,
  sonata-system-legacy-software,
  sonata-sim-boot-stub,
  cheriot-rtos-test-suite,
  sonata-simulator,
}: let
  inherit (pkgs.lib) fileset;
in {
  fpga = pkgs.writeShellApplication {
    name = "tests-fpga";
    runtimeInputs = [pythonEnv pkgs.openocd];
    text = ''
      set +u
      if [ -z "$1" ]; then
        echo "Please provide the tty device location (e.g. /dev/ttyUSB2)" \
          "as the first argument."
        exit 2
      fi
      echo "Sonata system test suite"
      ${../util/test_runner.py} -t 30 fpga "$1" \
        --elf-file ${sonata-system-software}/bin/test_runner \
        --tcl-file ${../util/sonata-openocd-cfg.tcl}
      echo "RTOS test suite"
      ${../util/test_runner.py} -t 600 fpga "$1" \
        --uf2-file ${cheriot-rtos-test-suite}/share/test-suite.uf2
    '';
  };

  simulator = pkgs.stdenvNoCC.mkDerivation {
    name = "tests-simulator";
    src = fileset.toSource {
      root = ./.; # an empty source directory
      fileset = fileset.unions [];
    };
    dontBuild = true;
    doCheck = true;
    buildInputs = [sonata-simulator pythonEnv];
    checkPhase = ''

      printf "Nix: Running legacy tests..."
      python ${../util/test_runner.py} -t 240 sim \
          --elf-file ${sonata-system-legacy-software}/bin/memory_test --options "+disable_cheri"
      echo "Nix: completed!"

      printf "Nix: Running CHERIoT tests..."
      python ${../util/test_runner.py} -t 240 sim \
          --elf-file ${sonata-system-software}/bin/test_runner
      echo "Nix: completed!"

      printf "Nix: Running CHERIoT RTOS tests..."
      python ${../util/test_runner.py} -t 600 sim \
          --sim-boot-stub ${sonata-sim-boot-stub.out}/share/sim_boot_stub \
          --elf-file ${cheriot-rtos-test-suite}/share/test-suite
      echo "Nix: completed!"
    '';
    installPhase = "mkdir $out";
  };
}
