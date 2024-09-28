# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Top generation circuitry using a top configuration and templates."""

import subprocess
from pathlib import Path
from typing import Iterator, NamedTuple, TypeAlias

from mako.template import Template

from .parser import BlockIoCombine, BlockIoType, TopConfig


class PinmuxIoToBlocks(NamedTuple):
    """Describes pinmux IO going to blocks"""

    direction: str
    width: str
    name: str
    instances: int


def ios_to_blocks_iter(config: TopConfig) -> Iterator[PinmuxIoToBlocks]:
    for block in config.blocks:
        instances = block.instances
        for io in block.ios:
            name = f"{block.name}_{io.name}"
            width = "" if io.length is None else f"[{io.length - 1}:0] "
            # Generate pinmux module input and outputs
            match io.type:
                case BlockIoType.OUTPUT:
                    yield PinmuxIoToBlocks(
                        "input ", width, name + "_i", instances
                    )
                case BlockIoType.INPUT:
                    yield PinmuxIoToBlocks(
                        "output", width, name + "_o", instances
                    )
                case BlockIoType.INOUT:
                    yield PinmuxIoToBlocks(
                        "input ", width, name + "_i", instances
                    )
                    yield PinmuxIoToBlocks(
                        "input ", width, name + "_en_i", instances
                    )
                    yield PinmuxIoToBlocks(
                        "output", width, name + "_o", instances
                    )


class PinmuxIoToPins(NamedTuple):
    """Describes pinmux IO going to pins"""

    width: str
    name: str


def ios_to_pins_iter(config: TopConfig) -> Iterator[PinmuxIoToPins]:
    return (
        PinmuxIoToPins(
            width="" if pin.length is None else f"[{pin.length - 1}:0] ",
            name=pin.name,
        )
        for pin in config.pins
    )


BlockIoId: TypeAlias = tuple[str, str]
"""
A tuple of block name and block IO name pair to
uniquely identify a block IO. i.e. (block_name, block_io_name)
"""
BlockIoConnectionMap: TypeAlias = dict[BlockIoId, list[list[list[str]]]]
"""
Maps a block IO to the pins it connects to.
block_io_pins[block_io_id][block_instance][index][connections] == pin_name
"""


def block_connections_map(config: TopConfig) -> BlockIoConnectionMap:
    block_connections: BlockIoConnectionMap = {}

    for block in config.blocks:
        instances = block.instances
        for io in block.ios:
            length = 1 if io.length is None else io.length
            block_connections[(block.name, io.name)] = [
                [[] for _ in range(length)] for _ in range(instances)
            ]

    for pin in config.pins:
        # Populate block parameters with which pins can connect to them.
        for pin_block_io in pin.block_ios:
            (block_io_name, index) = (
                ("ios", pin_block_io.io)
                if isinstance(pin_block_io.io, int)
                else (pin_block_io.io, 0)
            )
            block = config.get_block(pin_block_io.block)
            for block_io in block.ios:
                if block_io_name == block_io.name:
                    block_io_id: BlockIoId = (block.name, block_io_name)
                    if pin.length is not None:
                        for i in range(pin.length):
                            block_connections[block_io_id][
                                pin_block_io.instance
                            ][index + i].append(pin.name)
                    else:
                        block_connections[block_io_id][pin_block_io.instance][
                            index
                        ].append(pin.name)

    return block_connections


class BlockIoInfo(NamedTuple):
    """Describes a block IO"""

    block: str
    io: str
    instance: int
    bit_index_str: str
    is_inout: bool


PinToBlockOutputMap: TypeAlias = dict[str, list[BlockIoInfo]]
"""
Maps a pin names to a the block outputs that connect to it.
"""


def pin_to_block_output_map(
    config: TopConfig, block_connections: BlockIoConnectionMap
) -> PinToBlockOutputMap:
    pin_to_block_output_map: PinToBlockOutputMap = {}

    for block in config.blocks:
        for io in block.ios:
            if io.type in (BlockIoType.OUTPUT, BlockIoType.INOUT):
                connections = block_connections[(block.name, io.name)]
                for inst_idx, inst_pins in enumerate(connections):
                    for bit_idx, pins in enumerate(inst_pins):
                        bit_str = "" if len(inst_pins) == 1 else f"[{bit_idx}]"
                        for pin_name in pins:
                            pin = config.get_pin(pin_name)
                            pin_to_block_output_map.setdefault(
                                pin.name, []
                            ).append(
                                BlockIoInfo(
                                    block.name,
                                    io.name,
                                    inst_idx,
                                    bit_str,
                                    io.type == BlockIoType.INOUT,
                                )
                            )

    return pin_to_block_output_map


class InputPin(NamedTuple):
    block_input: str
    instances: int
    bit_idx: int
    bit_str: str
    pinmux_input_connections: list[str]


def input_pins_iter(
    config: TopConfig, block_connections: BlockIoConnectionMap
) -> Iterator[InputPin]:
    for block in config.blocks:
        for io in block.ios:
            if io.type == BlockIoType.INPUT or (
                io.type == BlockIoType.INOUT
                and io.combine == BlockIoCombine.MUX
            ):
                connections = block_connections[(block.name, io.name)]
                assert (
                    len(connections) > 0
                ), f"A {block.name}.{io.name} isn't connected to anything"
                for bit_idx in range(len(connections[0])):
                    for inst_idx in range(len(connections)):
                        inst_pins = connections[inst_idx]
                        pins = connections[inst_idx][bit_idx]

                        bit_str = "" if len(inst_pins) == 1 else f"_{bit_idx}"
                        default_value = f"1'b{io.default}"
                        input_pins = [default_value]
                        for pin_name in pins:
                            pin_with_idx = pin_name
                            pin = config.get_pin(pin_name)
                            # the pin is an array
                            if pin.length is not None:
                                assert isinstance(pin.block_ios[0].io, int), (
                                    f"Pin Array '{pin.name}' must be "
                                    "connected to a block io of type int."
                                )
                                pin_with_idx = (
                                    f"{pin_name}"
                                    f"[{bit_idx - pin.block_ios[0].io}]"
                                )
                            input_pins.append(pin_with_idx)
                        # Make sure there are always two values in the input
                        # list because the second one is always selected by
                        # default in the RTL.
                        if len(input_pins) == 1:
                            input_pins.append(default_value)
                        yield InputPin(
                            f"{block.name}_{io.name}",
                            inst_idx,
                            bit_idx,
                            bit_str,
                            input_pins,
                        )


class InOutPin(NamedTuple):
    block_input: str
    instance: int
    pins_to_combine: list[str]
    pin_selectors: list[int]
    combine_type: BlockIoCombine


def inout_pins_iter(
    config: TopConfig,
    block_connections: BlockIoConnectionMap,
    pin_to_block_outputs: PinToBlockOutputMap,
) -> Iterator[InOutPin]:
    for block in config.blocks:
        for io in block.ios:
            if io.type == BlockIoType.INOUT and io.combine in (
                BlockIoCombine.AND,
                BlockIoCombine.OR,
            ):
                connections = block_connections[(block.name, io.name)]
                for inst_idx, inst_pins in enumerate(connections):
                    assert len(inst_pins) == 1, (
                        "Currently we don't support indexing inout "
                        "signals that are combined through muxing."
                    )
                    combine_pins = inst_pins[0]
                    input_name = f"{block.name}_{io.name}"
                    combine_pin_selectors = []
                    for pin_name in combine_pins:
                        pin = config.get_pin(pin_name)
                        for sel_idx, block_output in enumerate(
                            pin_to_block_outputs[pin.name]
                        ):
                            assert block_output.bit_index_str == "", (
                                "Combining indexed pins is currently "
                                "unsupported."
                            )
                            if (
                                block_output.block,
                                block_output.io,
                                block_output.instance,
                            ) == (block.name, io.name, inst_idx):
                                combine_pin_selectors.append(
                                    1 << (sel_idx + 1)
                                )
                                break
                    assert len(combine_pins) == len(
                        combine_pin_selectors
                    ), "Could not fill combine pin selectors properly."
                    yield InOutPin(
                        input_name,
                        inst_idx,
                        combine_pins,
                        combine_pin_selectors,
                        io.combine,
                    )


class OutputPin(NamedTuple):
    pin_name: str
    idx_str: str
    """If pin array, array index after underscore e.g. '_1'"""
    idx_alt: str
    """If pin array, array index surrounded by square brackets e.g. '[1]'"""
    connected_block_io: list[BlockIoInfo]


def output_pins_iter(
    config: TopConfig, pin_to_block_outputs: PinToBlockOutputMap
) -> Iterator[OutputPin]:
    for pin in config.pins:
        if outputs := pin_to_block_outputs.get(pin.name):
            if pin.length is not None:
                assert pin.length == len(
                    outputs
                ), f"Arrayed pin '{pin.name}' must have complete mapping"
                for i in range(pin.length):
                    yield OutputPin(pin.name, f"_{i}", f"[{i}]", [outputs[i]])
            else:
                yield OutputPin(pin.name, "", "", outputs)


def generate_top(config: TopConfig) -> None:
    """Generate a top from a top configuration."""

    pinmux_ios_to_blocks = list(ios_to_blocks_iter(config))
    pinmux_ios_to_pins = list(ios_to_pins_iter(config))

    block_connections = block_connections_map(config)
    pin_to_block_outputs = pin_to_block_output_map(config, block_connections)

    input_pins = list(input_pins_iter(config, block_connections))
    output_pins = list(output_pins_iter(config, pin_to_block_outputs))
    inout_pins = list(
        inout_pins_iter(config, block_connections, pin_to_block_outputs)
    )

    # Then we use those parameters to generate our SystemVerilog using Mako
    template_variables = {
        "gpio_num": config.get_block("gpio").instances,
        "uart_num": config.get_block("uart").instances,
        "i2c_num": config.get_block("i2c").instances,
        "spi_num": config.get_block("spi").instances,
        "block_ios": pinmux_ios_to_blocks,
        "pin_ios": pinmux_ios_to_pins,
        "input_list": input_pins,
        "output_list": output_pins,
        "combine_list": inout_pins,
    }
    for template_file, output_file in (
        ("data/xbar_main.hjson", "data/xbar_main_generated.hjson"),
        (
            "rtl/templates/sonata_xbar_main.sv.tpl",
            "rtl/bus/sonata_xbar_main.sv",
        ),
        ("rtl/templates/sonata_pkg.sv.tpl", "rtl/system/sonata_pkg.sv"),
        ("rtl/templates/pinmux.sv.tpl", "rtl/system/pinmux.sv"),
        ("doc/ip/pinmux.md.tpl", "doc/ip/pinmux.md"),
    ):
        print("Generating from template: " + template_file)
        content = Template(filename=template_file).render(**template_variables)
        Path(output_file).write_text(content)

    subprocess.call(["sh", "util/generate_xbar.sh"])
