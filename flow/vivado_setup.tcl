# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

## Vivado project custom configuration script
##
## For configuration not supported by fusesoc/edalize

# Configure when and how the post-synthesis timing XDC file is read.
# The `Tcl` `file_type` makes Vivado use the `-unmanaged` argument with
# `read_xdc`, which allows us to use commands such as `foreach`.
# The `used_in_synthesis` property makes Vivado delay reading the file
# until after synthesis, allowing internal paths to be specified.
# Note: `file_type` must be set before `used_in_synthesis`.
set_property file_type Tcl [get_files impl_timing.xdc]
set_property used_in_synthesis false [get_files impl_timing.xdc]
