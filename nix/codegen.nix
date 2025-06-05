# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  zermio-cli,
}: {
  legacy-mmio = pkgs.writeShellApplication {
    name = "generate-legacy-mmio";
    runtimeInputs = [zermio-cli];
    text = ''
      zermio-cli import-svd --header-file=${../data/license_header.md} --svd=${../data/ibex.svd} export-cpp --dir=sw/legacy/hal/mmio/
      find sw/legacy/hal/mmio -name "*.hh" -print0  | xargs -0 clang-format -i
    '';
  };
}
