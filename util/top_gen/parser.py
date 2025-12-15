# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Top configuration structures and deserialiser from TOML files."""

from enum import Enum
from pathlib import Path

import toml
from pydantic import BaseModel, field_validator, model_validator
from pydantic.dataclasses import dataclass
from typing_extensions import Self


class Direction(str, Enum):
    INOUT = "inout"
    INPUT = "input"
    OUTPUT = "output"

    def __and__(self, other: "Direction") -> "Direction":
        match (self, other):
            case (Direction.INPUT, Direction.INPUT):
                return self
            case (Direction.OUTPUT, Direction.OUTPUT):
                return self
            case _:
                return Direction.INOUT


class BlockIoCombine(str, Enum):
    AND = "and"
    OR = "or"
    MUX = "mux"


class BlockIo(BaseModel, frozen=True):
    name: str
    type: Direction
    combine: BlockIoCombine | None = None
    default: int = 0
    length: int | None = None

    @model_validator(mode="after")
    def verify_block(self) -> Self:
        assert (
            self.type != Direction.INOUT or self.default == 0
        ), "An inout block IO cannot have a default value other than 0."
        return self


class Block(BaseModel, frozen=False):
    name: str
    instances: int
    ios: list[BlockIo]
    memory_start: int
    memory_size: int
    xbar: dict[str, str] = {}
    muxed_instances: int = 0

    @field_validator("instances")
    @staticmethod
    def check_instances(instances: int) -> int:
        if instances < 1:
            raise ValueError("Must have one or more instances of a block.")
        return instances


@dataclass(frozen=True)
class BlockIoUid:
    block: str
    instance: int
    io: str
    io_index: int | None = None
    """Used to index into an arrayed block IO."""


class Pin(BaseModel, frozen=True):
    name: str
    length: int | None = None
    """If not None, this is grouping of multiple pins with the given length."""
    block_ios: list[BlockIoUid]
    no_default_out: bool = False

    @model_validator(mode="after")
    def verify_pin(self) -> Self:
        if self.length is not None and not all(
            isinstance(block.io_index, int) for block in self.block_ios
        ):
            raise ValueError(
                "A pin grouping can only link to a block io array, i.e. "
                "when a length is specified all block_ios.io must be integers."
            )
        return self


class TopConfig(BaseModel, frozen=False):
    blocks: list[Block]
    pins: list[Pin]

    @field_validator("pins")
    @staticmethod
    def check_pins(pins: list[Pin]) -> list[Pin]:
        all_names = {pin.name for pin in pins}
        if len(all_names) != len(pins):
            raise ValueError("All pins must have unique names.")
        return pins

    def get_pin(self, name: str) -> Pin:
        try:
            return next(pin for pin in self.pins if pin.name == name)
        except StopIteration:
            print(f"A pin named '{name}' could not be found.")
            exit(3)


def parse_top_config(filename: Path) -> TopConfig:
    """Load the top configuration from the given TOML file."""
    with filename.open() as file:
        config_dict = toml.load(file)
    return TopConfig(**config_dict)
