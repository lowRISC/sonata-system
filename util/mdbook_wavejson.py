#!/usr/bin/env python3
# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""mdbook preprocessor for wavejson code-blocks.

It surrounds wavejson code-blocks with script tags of type WaveDrom.
"""

import json
import re
import sys

from mdbook import utils as md_utils

WAVEJSON_REG_FIXED_CONFIG = r"""
    {"fontsize": 10,
     "lanes": 1,
     "vspace": 140,
     "hspace": 800,
     "fontfamily": "sans-serif",
     "fontweight": "normal",
     "compact": false}"""

WAVEJSON_REG_PATTERN = re.compile("```wavejson_reg\n(.+?)\n```", re.DOTALL)
WAVEJSON_REG_REPLACE = (
    r'<script type="WaveDrom">{ "reg": \1, "config":'
    + WAVEJSON_REG_FIXED_CONFIG
    + r"}</script>"
)

WAVEJSON_PATTERN = re.compile("```wavejson\n(.+?)\n```", re.DOTALL)
WAVEJSON_REPLACE = r'<script type="WaveDrom">\1</script>'


def main() -> None:
    if len(sys.argv) > 2:
        if (sys.argv[1], sys.argv[2]) == (
            "supports",
            "html",
        ):
            sys.exit(0)
        else:
            sys.exit(1)

    # load both the context and the book from stdin
    context, book = json.load(sys.stdin)

    for chapter in md_utils.chapters(book["sections"]):
        chapter["content"] = WAVEJSON_REG_PATTERN.sub(
            WAVEJSON_REG_REPLACE, chapter["content"]
        )
        chapter["content"] = WAVEJSON_PATTERN.sub(
            WAVEJSON_REPLACE, chapter["content"]
        )

    # dump the book into stdout
    print(json.dumps(book))


if __name__ == "__main__":
    main()
