# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

add_library(hal INTERFACE)
target_include_directories(hal INTERFACE "./")
target_compile_options(hal INTERFACE "-O1")

