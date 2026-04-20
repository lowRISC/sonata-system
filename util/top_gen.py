#!/usr/bin/env python3
# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import subprocess
from pathlib import Path

from top_gen import generator, parser

RDL_GEN_FOLDER = Path("build/rdl")

if __name__ == "__main__":
    # Generates reg2hw, hw2reg and rdl.json from the rdl files.
    RDL_GEN_FOLDER.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["rdl2ot", "export-rtl", "--soc", "data/top.rdl", RDL_GEN_FOLDER],
        check=True,
    )

    config = parser.parse_top_config(
        Path("data/top_config.toml"), RDL_GEN_FOLDER / "rdl.json"
    )
    generator.generate_top(config)
