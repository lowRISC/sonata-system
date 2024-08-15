#!/usr/bin/env python3
# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

from mako.template import Template

gpio_num = 5
uart_num = 5
i2c_num  = 2
spi_num  = 7

xbar_spec = ('data/xbar_main.hjson', 'data/xbar_main_generated.hjson')
sonata_xbar_spec = ('rtl/templates/sonata_xbar_main.sv.tpl', 'rtl/bus/sonata_xbar_main.sv')
pkg_spec = ('rtl/templates/sonata_pkg.sv.tpl',       'rtl/system/sonata_pkg.sv')

specs = [xbar_spec, sonata_xbar_spec, pkg_spec]

for spec in specs:
    print('Generating from template: ' + spec[0])
    template = Template(filename=spec[0])
    content = template.render(gpio_num=gpio_num, uart_num=uart_num, i2c_num=i2c_num, spi_num=spi_num)
    with open(spec[1], 'w') as file:
        file.write(content)
