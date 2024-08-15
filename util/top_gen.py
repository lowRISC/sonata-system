#!/usr/bin/env python3.10
# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import toml
from pydantic import BaseModel
from enum import Enum
from mako.template import Template


class BlockIoType(str, Enum):
    IN_OUT = "inout"
    INPUT = "input"
    OUTPUT = "output"


class BlockIo(BaseModel):
    name: str
    type: BlockIoType
    combine: str | None = None
    default: int | None = None


class Block(BaseModel):
    name: str
    instances: int
    ios: list[BlockIo]


class PinIo(BaseModel):
    block: str
    instance: int
    io: str | int


class Pin(BaseModel):
    name: str
    length: int | None = None
    block_ios: list[PinIo]


class Config(BaseModel):
    blocks: list[Block]
    pins: list[Pin]


if __name__ == "__main__":
    with open("data/top_config.toml", "r") as file:
        config = Config(**toml.load(file))

    def get_block(name: str) -> Block:
        return next(block for block in config.blocks if block.name == name)

    try:
        gpio = get_block("gpio")
        uart = get_block("uart")
        i2c = get_block("i2c")
        spi = get_block("spi")
    except StopIteration:
        print("One or more blocks not present in configuration.")
        exit(3)

    # Then we use those parameters to generate our SystemVerilog using Mako
    xbar_spec = ("data/xbar_main.hjson", "data/xbar_main_generated.hjson")
    sonata_xbar_spec = (
        "rtl/templates/sonata_xbar_main.sv.tpl",
        "rtl/bus/sonata_xbar_main.sv",
    )
    pkg_spec = ("rtl/templates/sonata_pkg.sv.tpl", "rtl/system/sonata_pkg.sv")

    specs = [xbar_spec, sonata_xbar_spec, pkg_spec]

    for template_file, output_file in specs:
        print("Generating from template: " + template_file)
        template = Template(filename=template_file)
        content = template.render(
            gpio_num=gpio.instances,
            uart_num=uart.instances,
            i2c_num=i2c.instances,
            spi_num=spi.instances,
        )
        with open(output_file, "w") as file:
            file.write(content)
