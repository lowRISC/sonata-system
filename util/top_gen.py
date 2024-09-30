#!/usr/bin/env python3.10
# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from top_gen import generator, parser

if __name__ == "__main__":
    config = parser.parse_top_config(Path("data/top_config.toml"))
    generator.generate_top(config)
