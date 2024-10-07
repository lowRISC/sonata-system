#!/usr/bin/env python3.10
# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
from typing import NamedTuple

from mako.template import Template


class SystemInfo(NamedTuple):
    commit_hash: str
    dirty: bool


def generate_system_info() -> None:
    commit = os.environ.get("FLAKE_GIT_COMMIT")
    dirty = bool(os.environ.get("FLAKE_GIT_DIRTY"))
    if commit is None or dirty is None:
        import git

        try:
            repo = git.Repo(search_parent_directories=True)
            commit = repo.head.object.hexsha
            dirty = repo.is_dirty()
        except git.exc.InvalidGitRepositoryError:
            print("Error: Couldn't get commit and dirty via git or Nix")
            exit()
    system_info = SystemInfo(commit[0:16], dirty)

    print(
        "System info: Git hash "
        + system_info.commit_hash
        + " "
        + ("DIRTY!" if system_info.dirty else "clean")
    )
    system_info_dir = Path(__file__).parents[1]

    for template, output in (
        (
            system_info_dir / "templates" / "system_info.sv.tpl",
            Path("system_info.sv"),
        ),
        (
            system_info_dir / "templates" / "system_info.core.tpl",
            Path("system_info.core"),
        ),
        # TODO is there a way not to have to copy these over to the
        # auto-generated folder?
        (
            system_info_dir / "rtl" / "system_info_reg_pkg.sv",
            Path("system_info_reg_pkg.sv"),
        ),
        (
            system_info_dir / "rtl" / "system_info_reg_top.sv",
            Path("system_info_reg_top.sv"),
        ),
    ):
        content = Template(filename=str(template)).render(
            system_info=system_info
        )
        output.write_text(content)


def main() -> None:
    generate_system_info()


if __name__ == "__main__":
    main()
