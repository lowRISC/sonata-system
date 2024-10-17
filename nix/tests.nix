# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  pythonEnv,
  sonata-system-software,
  sonata-sim-boot-stub,
  cheriot-rtos-test-suite,
  sonata-simulator,
}: {
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
    '';
  };

  simulator = pkgs.stdenvNoCC.mkDerivation {
    name = "tests-simulator";
    src = ./.;
    dontBuild = true;
    doCheck = true;
    buildInputs = [sonata-simulator pythonEnv];
    checkPhase = ''
      python ${../util/test_runner.py} -t 60 sim \
          --elf-file ${sonata-system-software}/bin/test_runner
      echo "Test runner complete!"
      python ${../util/test_runner.py} -t 600 sim \
          --sim-boot-stub ${sonata-sim-boot-stub.out}/share/sim_boot_stub \
          --elf-file ${cheriot-rtos-test-suite}/share/test-suite
    '';
    installPhase = "mkdir $out";
  };
}
