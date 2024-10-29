# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Top generation circuitry using a top configuration and templates."""

import functools
import subprocess
from pathlib import Path
from typing import Iterator, NamedTuple, TypeAlias

from mako.template import Template
from pydantic import BaseModel
from pydantic.dataclasses import dataclass

from .parser import (
    Block,
    BlockIoCombine,
    BlockIoUid,
    Direction,
    Pin,
    TopConfig,
)


class BlockIoFlat(BaseModel, frozen=True):
    """Describes an IO between the pinmux and a block.

    Flat meaning this describes individual block IOs,
    splitting up arrayed block IOs.
    """

    uid: BlockIoUid

    default_value: int
    direction: Direction
    combine: BlockIoCombine | None = None

    @property
    def is_inout(self) -> bool:
        return self.direction == Direction.INOUT

    @property
    def name(self) -> str:
        uid = self.uid
        suffix = f"_{uid.io_index}" if uid.io_index is not None else ""
        return f"{uid.block}_{uid.io}_{uid.instance}{suffix}"

    @property
    def io_idx_str(self) -> str:
        return (
            f"[{self.uid.io_index}]" if self.uid.io_index is not None else ""
        )

    @property
    def doc_name(self) -> str:
        uid = self.uid
        return f"{uid.block}[{uid.instance}].{uid.io}{self.io_idx_str}"


@dataclass(frozen=True)
class PinFlat:
    """Describes an IO between the pinmux and the pins.

    Flat meaning this describes individual Pins, splitting up grouped pins.
    """

    group_name: str
    block_io_links: list[BlockIoUid]

    direction: Direction
    group_index: int | None = None

    @property
    def name(self) -> str:
        if self.group_index is None:
            return f"{self.group_name}"
        else:
            return f"{self.group_name}_{self.group_index}"

    @property
    def doc_name(self) -> str:
        if self.group_index is None:
            return f"{self.group_name}"
        else:
            return f"{self.group_name}[{self.group_index}]"

    @property
    def direction_prefix(self) -> str:
        match self.direction:
            case Direction.INPUT:
                return "in_"
            case Direction.OUTPUT:
                return "out_"
            case Direction.INOUT:
                return "inout_"

    @property
    def idx_param(self) -> str:
        return f"{self.direction_prefix}PIN_{self.name}".upper()

    def is_input(self) -> bool:
        return self.direction == Direction.INPUT

    def is_output(self) -> bool:
        return self.direction == Direction.OUTPUT

    def is_inout(self) -> bool:
        return self.direction == Direction.INOUT


BlockIoToPinsMap: TypeAlias = dict[BlockIoUid, list[PinFlat]]
"""Maps a block name to a list of pins it connects to."""


class OutputBlockIo(NamedTuple):
    """Describes an output to a block IO for the template."""

    block_io: BlockIoFlat
    possible_pins: list[PinFlat]
    num_options: int


class OutputPin(NamedTuple):
    """Describes an output to a pin for the template."""

    pin: PinFlat
    possible_block_outputs: list[BlockIoFlat]
    num_options: int


class CombinedInputBlockIo(NamedTuple):
    """Describes an input block IO that combines multiple pin inputs."""

    block_io: BlockIoFlat
    default_value: str
    operator: str
    pins_and_select_values: list[tuple[PinFlat, int]]


def flatten_block_ios(blocks: list[Block]) -> Iterator[BlockIoFlat]:
    for block in blocks:
        for instance in range(block.instances):
            for io in block.ios:
                if io.length is None:
                    yield BlockIoFlat(
                        uid=BlockIoUid(
                            block.name,
                            instance,
                            io.name,
                        ),
                        default_value=io.default,
                        direction=io.type,
                        combine=io.combine,
                    )
                else:
                    for io_index in range(io.length):
                        yield BlockIoFlat(
                            uid=BlockIoUid(
                                block.name,
                                instance,
                                io.name,
                                io_index,
                            ),
                            default_value=io.default,
                            direction=io.type,
                            combine=io.combine,
                        )


def flatten_pins(
    pins: list[Pin], block_ios: list[BlockIoFlat]
) -> Iterator[PinFlat]:
    block_ios_direction: dict[BlockIoUid, Direction] = {
        bio.uid: bio.direction for bio in block_ios
    }

    def pin_direction(pin: Pin) -> Direction:
        return functools.reduce(
            Direction.__and__,
            (
                block_ios_direction[block_io_uid]
                for block_io_uid in pin.block_ios
            ),
        )

    for pin in pins:
        direction = pin_direction(pin)
        if pin.length is None:
            yield PinFlat(pin.name, pin.block_ios, direction)
        else:
            for group_index in range(pin.length):
                block_io_links = [
                    BlockIoUid(
                        block_io.block,
                        block_io.instance,
                        block_io.io,
                        block_io.io_index + group_index,
                    )
                    for block_io in pin.block_ios
                    # This is checked at validation time
                    if isinstance(block_io.io_index, int)
                ]
                yield PinFlat(pin.name, block_io_links, direction, group_index)


def block_io_to_pin_map(
    blocks: list[BlockIoFlat], pins: list[PinFlat]
) -> BlockIoToPinsMap:
    mapping: BlockIoToPinsMap = {block_io.uid: [] for block_io in blocks}
    for pin in pins:
        for link in pin.block_io_links:
            mapping[link].append(pin)
    return mapping


def output_block_ios_iter(
    block_ios: list[BlockIoFlat], block_io_to_pins: BlockIoToPinsMap
) -> Iterator[OutputBlockIo]:
    for block_io in block_ios:
        if block_io.direction != Direction.INPUT and (
            block_io.direction != Direction.INOUT
            or block_io.combine != BlockIoCombine.MUX
        ):
            continue

        possible_pins = block_io_to_pins[block_io.uid]

        yield OutputBlockIo(
            block_io,
            possible_pins,
            max(len(possible_pins) + 1, 2),
        )


def output_pins_iter(
    pins: list[PinFlat], block_ios: list[BlockIoFlat]
) -> Iterator[OutputPin]:
    blocks_map: dict[BlockIoUid, BlockIoFlat] = {
        bio.uid: bio for bio in block_ios
    }
    for pin in pins:
        # Remove block inputs and convert to BlockIoUid to BlockIoFlat
        block_outputs = [
            blocks_map[block_io_uid]
            for block_io_uid in pin.block_io_links
            if blocks_map[block_io_uid].direction != Direction.INPUT
        ]
        if len(block_outputs) > 0:
            yield OutputPin(pin, block_outputs, len(block_outputs) + 1)


def combined_input_block_ios_iter(
    block_ios: list[BlockIoFlat], block_io_to_pins: BlockIoToPinsMap
) -> Iterator[CombinedInputBlockIo]:
    for block_io in block_ios:
        match (block_io.direction, block_io.combine):
            case (Direction.INOUT, BlockIoCombine.AND):
                default_value = "1'b1"
                operator = " &"
            case (Direction.INOUT, BlockIoCombine.OR):
                default_value = "1'b0"
                operator = " |"
            case _:
                continue

        pins = block_io_to_pins[block_io.uid]
        pins_and_select_values = [
            (pin, 1 << (select_idx + 1))
            for pin in pins
            for select_idx, block_io_id in enumerate(pin.block_io_links)
            if block_io.uid == block_io_id
        ]

        assert len(pins) == len(
            pins_and_select_values
        ), "Could not fill combine pin selectors properly."

        yield CombinedInputBlockIo(
            block_io,
            default_value,
            operator,
            pins_and_select_values,
        )


def block_port_definitions(block: Block) -> Iterator[str]:
    instances_param = f"{block.name.upper()}_NUM"
    for io in block.ios:
        name = f"{block.name}_{io.name}"
        width = "" if io.length is None else f"[{io.length - 1}:0] "
        match io.type:
            case Direction.INPUT:
                yield f"output {width}{name}_o[{instances_param}]"
            case Direction.OUTPUT:
                yield f"input  {width}{name}_i   [{instances_param}]"
                yield f"input  {width}{name}_en_i[{instances_param}]"
            case Direction.INOUT:
                yield f"output {width}{name}_o   [{instances_param}]"
                yield f"input  {width}{name}_i   [{instances_param}]"
                yield f"input  {width}{name}_en_i[{instances_param}]"


def generate_top(config: TopConfig) -> None:
    """Generate a top from a top configuration."""

    block_ios = list(flatten_block_ios(config.blocks))
    pins = list(flatten_pins(config.pins, block_ios))

    block_io_to_pins = block_io_to_pin_map(block_ios, pins)

    template_variables = {
        "config": config,
        "block_port_definitions": block_port_definitions,
        "in_pins": list(filter(PinFlat.is_input, pins)),
        "out_pins": list(filter(PinFlat.is_output, pins)),
        "inout_pins": list(filter(PinFlat.is_inout, pins)),
        "output_pins": list(output_pins_iter(pins, block_ios)),
        "output_block_ios": list(
            output_block_ios_iter(block_ios, block_io_to_pins)
        ),
        "combined_input_block_ios": list(
            combined_input_block_ios_iter(block_ios, block_io_to_pins)
        ),
    }
    for template_file, output_file in (
        ("data/xbar_main.hjson", "data/xbar_main_generated.hjson"),
        (
            "rtl/templates/sonata_xbar_main.sv.tpl",
            "rtl/bus/sonata_xbar_main.sv",
        ),
        ("rtl/templates/sonata_pkg.sv.tpl", "rtl/system/sonata_pkg.sv"),
        ("rtl/templates/pinmux.sv.tpl", "rtl/system/pinmux.sv"),
        ("util/templates/pinmux.md.tpl", "doc/ip/pinmux/README.md"),
        (
            "sw/cheri/common/platform-pinmux.hh.tpl",
            "sw/cheri/common/platform-pinmux.hh",
        ),
    ):
        print("Generating from template:", template_file)
        content = Template(filename=template_file).render(**template_variables)
        Path(output_file).write_text(str(content))

    try:
        subprocess.run(["sh", "util/generate_xbar.sh"], check=True)
        subprocess.run(
            ["clang-format", "sw/cheri/common/platform-pinmux.hh", "-i"],
            check=True,
        )
    except subprocess.CalledProcessError as err:
        exit(err.returncode)
