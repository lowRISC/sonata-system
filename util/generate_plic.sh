# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

rm -r rtl/system/autogen/rv_plic/
vendor/lowrisc_ip/util/ipgen.py generate \
  -C vendor/lowrisc_ip/ip_templates/rv_plic/ \
  -c data/rv_plic_cfg.hjson -o rtl/system/autogen/rv_plic
rm -r rtl/system/autogen/rv_plic/doc
rm -r rtl/system/autogen/rv_plic/fpv
rm rtl/system/autogen/rv_plic/README.md
